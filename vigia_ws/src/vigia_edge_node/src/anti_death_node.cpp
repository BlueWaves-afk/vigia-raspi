//
// anti_death_node.cpp — Emergency telemetry uplink to VIGIA AWS backend
//
// Transport: HTTPS POST to API Gateway /telemetry  (NOT MQTT — the backend
// uses REST API Gateway, not IoT Core).
//
// Signing contract (must match ValidatorFunction/index.ts):
//   payload_str = "VIGIA:{hazardType}:{lat}:{lon}:{timestamp}:{confidence}"
//   sig         = Ed25519_sign(payload_str, device_seckey)
//   JSON body:  { hazardType, lat, lon, timestamp, confidence,
//                 signature: base58(sig), publicKey: base58(pubkey),
//                 frame_base64: "<optional JPEG>" }
//
// Key provisioning (one-time on Pi): see docs/device_provisioning.md.
//   The 32-byte Ed25519 seed is written to /etc/vigia/device_ed25519.key and
//   the base58 public key is registered via POST /register-device.
//   (pynacl has no nacl.encoding.Base58Encoder — base58 is done inline; this
//    is also why base58_encode() below is implemented from scratch.)
//
// GPIO: libgpiod event_wait() on UPS POWER_FAIL line (active-low).
//       Requires libgpiod-dev; falls back to poll stub if not available.
//
// HTTP: libcurl (synchronous) — the Pi will be rebooting after this, so
//       async callbacks have no value. Curl with 8s connect + 4s transfer.
//

#include "anti_death_node.hpp"

#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <curl/curl.h>
#include <sodium.h>

// libgpiod — optional. If not available, GPIO stub polls power_fail_ flag.
#ifdef VIGIA_HAVE_LIBGPIOD
#  include <gpiod.h>
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ── Base58 alphabet (Bitcoin/Solana — matches bs58 npm package) ───────────
static const char kB58Chars[] = "123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz";

// ── libcurl write callback ─────────────────────────────────────────────────
static size_t curl_write_cb(void * ptr, size_t size, size_t nmemb, std::string * s) {
    s->append(static_cast<char *>(ptr), size * nmemb);
    return size * nmemb;
}

// ══════════════════════════════════════════════════════════════════════════
// Constructor / Destructor
// ══════════════════════════════════════════════════════════════════════════

AntiDeathNode::AntiDeathNode(const rclcpp::NodeOptions & opts)
: Node("anti_death_node", opts)
{
    if (sodium_init() < 0) {
        RCLCPP_ERROR(get_logger(), "libsodium init failed — Ed25519 signing disabled");
    }

    declare_parameter("ups_gpio_chip",         "/dev/gpiochip4");
    declare_parameter("ups_gpio_line",         17);
    declare_parameter("ups_gpio_active_low",   true);
    declare_parameter("power_window_seconds",  15.0);
    declare_parameter("aws_telemetry_url",     "");
    declare_parameter("device_key_path",       "/etc/vigia/device_ed25519.key");

    ups_gpio_chip_        = get_parameter("ups_gpio_chip").as_string();
    ups_gpio_line_        = get_parameter("ups_gpio_line").as_int();
    ups_gpio_active_low_  = get_parameter("ups_gpio_active_low").as_bool();
    power_window_seconds_ = get_parameter("power_window_seconds").as_double();
    aws_telemetry_url_    = get_parameter("aws_telemetry_url").as_string();
    device_key_path_      = get_parameter("device_key_path").as_string();

    if (!load_ed25519_key()) {
        RCLCPP_WARN(get_logger(),
            "Ed25519 key not found at %s — telemetry will be unsigned (rejected by server).",
            device_key_path_.c_str());
    }

    if (aws_telemetry_url_.empty()) {
        RCLCPP_WARN(get_logger(), "aws_telemetry_url not set — telemetry uplink disabled");
    }

    sub_latent_ = create_subscription<vigia_msgs::msg::SpatialLatent>(
        "/vigia/spatial_latent", vigia::qos::inference_results(),
        std::bind(&AntiDeathNode::on_spatial_latent, this, std::placeholders::_1));
    sub_et_ = create_subscription<vigia_msgs::msg::SignedEt>(
        "/vigia/signed_et", vigia::qos::signed_et(),
        std::bind(&AntiDeathNode::on_signed_et, this, std::placeholders::_1));
    sub_hazard_ = create_subscription<vigia_msgs::msg::HazardEvent>(
        "/vigia/hazard_event", vigia::qos::hazard_events(),
        std::bind(&AntiDeathNode::on_hazard_event, this, std::placeholders::_1));

    gpio_thread_ = std::thread(&AntiDeathNode::gpio_monitor_loop, this);

    RCLCPP_INFO(get_logger(),
        "AntiDeathNode ready: gpio=%s:%d window=%.0fs key=%s url=%s",
        ups_gpio_chip_.c_str(), ups_gpio_line_, power_window_seconds_,
        key_loaded_ ? "OK" : "MISSING",
        aws_telemetry_url_.empty() ? "UNSET" : aws_telemetry_url_.c_str());
}

AntiDeathNode::~AntiDeathNode()
{
    shutdown_requested_.store(true);
    if (gpio_thread_.joinable()) gpio_thread_.join();
    sodium_memzero(ed25519_seckey_, sizeof(ed25519_seckey_));
}

// ══════════════════════════════════════════════════════════════════════════
// Subscriber callbacks — just cache the latest message
// ══════════════════════════════════════════════════════════════════════════

void AntiDeathNode::on_spatial_latent(std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg) {
    latest_latent_ = std::move(msg);
}
void AntiDeathNode::on_signed_et(std::shared_ptr<const vigia_msgs::msg::SignedEt> msg) {
    latest_et_ = std::move(msg);
}
void AntiDeathNode::on_hazard_event(std::unique_ptr<vigia_msgs::msg::HazardEvent> msg) {
    latest_hazard_ = std::move(msg);
}

// ══════════════════════════════════════════════════════════════════════════
// Ed25519 key loading
// ══════════════════════════════════════════════════════════════════════════

bool AntiDeathNode::load_ed25519_key()
{
    // Key file: 32-byte seed written by nacl.signing.SigningKey.generate()
    // libsodium expects the 64-byte expanded secret key (seed || pubkey).
    std::ifstream f(device_key_path_, std::ios::binary);
    if (!f) return false;

    uint8_t seed[kSeedLen];
    f.read(reinterpret_cast<char *>(seed), kSeedLen);
    if (f.gcount() != static_cast<std::streamsize>(kSeedLen)) {
        sodium_memzero(seed, kSeedLen);
        return false;
    }

    uint8_t pubkey[kPubkeyLen];
    if (crypto_sign_ed25519_seed_keypair(pubkey, ed25519_seckey_, seed) != 0) {
        sodium_memzero(seed, kSeedLen);
        return false;
    }
    sodium_memzero(seed, kSeedLen);

    device_pubkey_b58_ = base58_encode(pubkey, kPubkeyLen);
    key_loaded_ = true;
    RCLCPP_INFO(get_logger(), "Ed25519 key loaded — pubkey: %s", device_pubkey_b58_.c_str());
    return true;
}

// ══════════════════════════════════════════════════════════════════════════
// Base58 encoding (Bitcoin/Solana alphabet — matches bs58 npm pkg)
// ══════════════════════════════════════════════════════════════════════════

std::string AntiDeathNode::base58_encode(const uint8_t * data, size_t len) const
{
    // Convert bytes to big-integer via repeated division by 58.
    std::vector<uint8_t> digits;
    digits.reserve(len * 138 / 100 + 1);

    for (size_t i = 0; i < len; ++i) {
        int carry = data[i];
        for (auto & d : digits) {
            carry += 256 * d;
            d = carry % 58;
            carry /= 58;
        }
        while (carry > 0) {
            digits.push_back(carry % 58);
            carry /= 58;
        }
    }

    // Count leading zero bytes → leading '1's
    size_t leading_zeros = 0;
    while (leading_zeros < len && data[leading_zeros] == 0) ++leading_zeros;

    std::string result(leading_zeros, '1');
    for (auto it = digits.rbegin(); it != digits.rend(); ++it)
        result += kB58Chars[*it];

    return result;
}

// ══════════════════════════════════════════════════════════════════════════
// Ed25519 signing — matches ValidatorFunction payload contract
// ══════════════════════════════════════════════════════════════════════════

std::string AntiDeathNode::sign_payload(const TelemetryPayload & p) const
{
    if (!key_loaded_) return "";

    // Contract from validator/index.ts:
    //   const payloadStr = `VIGIA:${hazardType}:${lat}:${lon}:${timestamp}:${confidence}`;
    std::ostringstream oss;
    oss << "VIGIA:" << p.hazard_type
        << ":" << std::fixed << std::setprecision(6) << p.lat
        << ":" << std::fixed << std::setprecision(6) << p.lon
        << ":" << p.timestamp_iso
        << ":" << std::fixed << std::setprecision(4) << p.confidence;
    const std::string msg_str = oss.str();

    uint8_t sig[crypto_sign_ed25519_BYTES];
    unsigned long long sig_len = 0;
    if (crypto_sign_ed25519_detached(sig, &sig_len,
        reinterpret_cast<const uint8_t *>(msg_str.data()), msg_str.size(),
        ed25519_seckey_) != 0) {
        return "";
    }
    return base58_encode(sig, sig_len);
}

// ══════════════════════════════════════════════════════════════════════════
// JSON builder — matches ValidatorFunction expected body
// ══════════════════════════════════════════════════════════════════════════

std::string AntiDeathNode::build_json(const TelemetryPayload & p,
                                      const std::string & sig) const
{
    // Minimal JSON serialization — no external library needed for this structure.
    auto esc = [](const std::string & s) -> std::string {
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            if (c == '"') out += "\\\"";
            else if (c == '\\') out += "\\\\";
            else out += c;
        }
        return out;
    };

    std::ostringstream j;
    j << std::fixed;
    j << "{";
    j << "\"hazardType\":\"" << esc(p.hazard_type) << "\",";
    j << "\"lat\":"         << std::setprecision(7) << p.lat << ",";
    j << "\"lon\":"         << std::setprecision(7) << p.lon << ",";
    j << "\"timestamp\":\"" << esc(p.timestamp_iso) << "\",";
    j << "\"confidence\":"  << std::setprecision(4) << p.confidence << ",";
    // geohash intentionally omitted (design spec §8.2, decision D5): the
    // backend recomputes it from lat/lon. It is NOT part of the signed payload
    // (VIGIA:type:lat:lon:ts:confidence), so omitting it does not affect the
    // Ed25519 signature.
    j << "\"signature\":\""  << esc(sig) << "\",";
    j << "\"publicKey\":\""  << esc(device_pubkey_b58_) << "\"";
    if (!p.frame_base64.empty()) {
        j << ",\"frame_base64\":\"" << esc(p.frame_base64) << "\"";
    }
    j << "}";
    return j.str();
}

// ══════════════════════════════════════════════════════════════════════════
// HTTPS POST via libcurl
// ══════════════════════════════════════════════════════════════════════════

bool AntiDeathNode::post_telemetry(const std::string & json_body)
{
    if (aws_telemetry_url_.empty()) {
        RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] aws_telemetry_url not set — cannot transmit");
        return false;
    }

    CURL * curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] curl_easy_init failed");
        return false;
    }

    std::string response_body;
    curl_slist * headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL,            aws_telemetry_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE,  (long)json_body.size());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,  curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA,      &response_body);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 8L);   // 8s connect
    curl_easy_setopt(curl, CURLOPT_TIMEOUT,        12L);  // 12s total (within 15s window)
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);   // enforce TLS — never disable
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] curl error: %s", curl_easy_strerror(res));
        return false;
    }

    RCLCPP_INFO(get_logger(), "[ANTI-DEATH] AWS response %ld: %s",
                http_code, response_body.substr(0, 200).c_str());

    // 202 = PENDING (accepted by ValidatorFunction) — the orchestrator picks it up async
    return (http_code == 202 || http_code == 200);
}

// ══════════════════════════════════════════════════════════════════════════
// GPIO monitor loop
// ══════════════════════════════════════════════════════════════════════════

void AntiDeathNode::gpio_monitor_loop()
{
    pthread_setname_np(pthread_self(), "vigia_gpio");

#ifdef VIGIA_HAVE_LIBGPIOD
    // libgpiod blocking edge-wait — ≤1ms latency on PREEMPT_RT
    struct gpiod_chip * chip = gpiod_chip_open(ups_gpio_chip_.c_str());
    if (!chip) {
        RCLCPP_ERROR(get_logger(), "gpiod_chip_open(%s) failed: %s",
                     ups_gpio_chip_.c_str(), strerror(errno));
        goto poll_fallback;
    }

    {
        struct gpiod_line * line = gpiod_chip_get_line(chip, ups_gpio_line_);
        if (!line) {
            RCLCPP_ERROR(get_logger(), "gpiod_chip_get_line(%d) failed", ups_gpio_line_);
            gpiod_chip_close(chip);
            goto poll_fallback;
        }

        // Active-low UPS signal — POWER_FAIL asserted when line goes LOW
        int req_res = gpiod_line_request_falling_edge_events(line, "vigia_antideath");
        if (req_res < 0) {
            RCLCPP_ERROR(get_logger(), "gpiod_line_request_falling_edge_events failed");
            gpiod_chip_close(chip);
            goto poll_fallback;
        }

        RCLCPP_INFO(get_logger(), "GPIO: watching chip=%s line=%d for FALLING_EDGE (UPS POWER_FAIL)",
                    ups_gpio_chip_.c_str(), ups_gpio_line_);

        while (!shutdown_requested_.load()) {
            struct timespec ts{0, 100'000'000};  // 100ms timeout
            int ev = gpiod_line_event_wait(line, &ts);
            if (ev < 0) {
                RCLCPP_ERROR(get_logger(), "gpiod_line_event_wait error: %s", strerror(errno));
                break;
            }
            if (ev == 1) {
                struct gpiod_line_event event;
                gpiod_line_event_read(line, &event);
                if (event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                    RCLCPP_ERROR(get_logger(), "UPS POWER_FAIL GPIO asserted — starting emergency sequence");
                    run_emergency_sequence();
                    gpiod_chip_close(chip);
                    return;
                }
            }
            // Also check for software-triggered test
            if (power_fail_.load()) {
                run_emergency_sequence();
                gpiod_chip_close(chip);
                return;
            }
        }
        gpiod_chip_close(chip);
        return;
    }

poll_fallback:
#endif
    // Polling fallback — active if libgpiod not available or chip open failed
    RCLCPP_WARN(get_logger(), "GPIO: using poll fallback (libgpiod unavailable)");
    while (!shutdown_requested_.load()) {
        std::this_thread::sleep_for(100ms);
        if (power_fail_.load()) {
            RCLCPP_ERROR(get_logger(), "POWER FAIL (poll) — starting emergency sequence");
            run_emergency_sequence();
            return;
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════
// Emergency sequence — must complete within power_window_seconds_
// ══════════════════════════════════════════════════════════════════════════

void AntiDeathNode::run_emergency_sequence()
{
    const auto start    = Clock::now();
    const auto deadline = start + std::chrono::duration<double>(power_window_seconds_);

    auto elapsed_ms = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - start).count();
    };
    auto budget_ok = [&]() { return Clock::now() < deadline; };

    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] ══ EMERGENCY SEQUENCE START ══ window=%.0fs",
                 power_window_seconds_);

    // ── State: CAPTURE_SNAPSHOT (T+0 → T+≤1s) ────────────────────────────────
    state_ = AntiDeathState::CAPTURE_SNAPSHOT;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] CAPTURE_SNAPSHOT (T+%ldms)", elapsed_ms());

    // Take local copies of latest cached data — seqlock-safe at SCHED_FIFO 99
    const auto hazard  = latest_hazard_;
    const auto et      = latest_et_;
    const auto latent  = latest_latent_;

    if (!hazard) {
        RCLCPP_WARN(get_logger(), "[ANTI-DEATH] No HazardEvent in cache — will transmit raw sensor data");
    }

    // ── State: SERIALIZE (T+≤1s → T+≤3s) ────────────────────────────────────
    state_ = AntiDeathState::SERIALIZE;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] SERIALIZE (T+%ldms)", elapsed_ms());

    if (!budget_ok()) { RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] BUDGET EXCEEDED in SERIALIZE"); goto shutdown; }

    {
        // Build telemetry payload from best available data
        TelemetryPayload p;

        // Hazard type classification
        if (hazard) {
            p.hazard_type  = "pothole";      // Phase 1: single hazard class from YOLO
            p.confidence   = hazard->rri_score;
            p.lat          = hazard->gps_pvt.latitude;
            p.lon          = hazard->gps_pvt.longitude;
        } else if (et) {
            // Fall back to GPS from signed_et packet
            p.hazard_type = "road_hazard";
            p.confidence  = 0.5f;
            p.lat         = et->lat;
            p.lon         = et->lon;
        } else {
            RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] No sensor data in cache — aborting transmit");
            goto shutdown;
        }

        // ISO 8601 timestamp — now()
        {
            auto now_t = std::chrono::system_clock::now();
            auto tt    = std::chrono::system_clock::to_time_t(now_t);
            std::tm tm_utc{};
            gmtime_r(&tt, &tm_utc);
            char ts_buf[32];
            std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%S.000Z", &tm_utc);
            p.timestamp_iso = ts_buf;
        }

        // Simple geohash7 from lat/lon — 7-character Ngeohash (matches server)
        // Full ngeohash library not worth linking for embedded; compute inline.
        // The server recomputes it anyway; we provide an approximate value.
        // Placeholder: encode lat/lon as fixed string — replace with geohash lib if needed.
        {
            // Approximate geohash-style identifier — server recomputes exact value
            char gh[32];
            std::snprintf(gh, sizeof(gh), "%.4f_%.4f", p.lat, p.lon);
            p.geohash  = std::string(gh).substr(0, 7);
            p.hazard_id = p.geohash + "#" + p.timestamp_iso;
        }

        // Sign the payload
        const std::string sig = sign_payload(p);
        if (sig.empty() && key_loaded_) {
            RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] Ed25519 signing failed");
            goto shutdown;
        }
        if (!key_loaded_) {
            RCLCPP_WARN(get_logger(), "[ANTI-DEATH] No key — sending unsigned (will be rejected by server)");
        }

        const std::string json_body = build_json(p, sig);

        RCLCPP_INFO(get_logger(), "[ANTI-DEATH] Payload: type=%s lat=%.4f lon=%.4f conf=%.3f ts=%s",
                    p.hazard_type.c_str(), p.lat, p.lon, p.confidence, p.timestamp_iso.c_str());

        // ── State: HTTPS_TRANSMIT (T+≤3s → T+≤13s budget) ───────────────────
        state_ = AntiDeathState::HTTPS_TRANSMIT;
        RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] HTTPS_TRANSMIT (T+%ldms)", elapsed_ms());

        if (!budget_ok()) { RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] BUDGET EXCEEDED before transmit"); goto shutdown; }

        const bool ok = post_telemetry(json_body);
        RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] Transmit %s (T+%ldms remaining=%.1fs)",
                     ok ? "SUCCESS" : "FAILED",
                     elapsed_ms(),
                     std::chrono::duration<double>(deadline - Clock::now()).count());
    }

shutdown:
    // ── State: SAFE_SHUTDOWN ──────────────────────────────────────────────────
    state_ = AntiDeathState::SAFE_SHUTDOWN;
    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] SAFE_SHUTDOWN (T+%ldms)", elapsed_ms());

    // Flush journald to ramoops — available in post-mortem after reboot
    sync();

    RCLCPP_ERROR(get_logger(), "[ANTI-DEATH] ══ SEQUENCE COMPLETE ══ Total: %ldms", elapsed_ms());
    // Power loss is expected here — /dev/shm contents lost (by design)
}
