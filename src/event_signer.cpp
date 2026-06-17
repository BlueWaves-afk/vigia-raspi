#include "event_signer.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#if defined(VIGIA_HAVE_OPENSSL)
#include <openssl/evp.h>
#include <openssl/hmac.h>
#endif

namespace vigia {

namespace {

std::string formatObservedAtIso8601()
{
    const auto now = std::chrono::system_clock::now();
    const auto secs =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) -
        std::chrono::duration_cast<std::chrono::milliseconds>(secs);

    const std::time_t t = secs.count();
    std::tm tm_buf{};
#if defined(_WIN32)
    gmtime_s(&tm_buf, &t);
#else
    gmtime_r(&t, &tm_buf);
#endif

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);

    std::ostringstream oss;
    oss << buf << '.' << std::setw(3) << std::setfill('0') << ms.count() << 'Z';
    return oss.str();
}

#if defined(VIGIA_HAVE_OPENSSL)
std::string base64Encode(const uint8_t* data, std::size_t len)
{
    static const char* kTable =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string out;
    out.reserve(((len + 2) / 3) * 4);

    for (std::size_t i = 0; i < len; i += 3) {
        const uint32_t octet_a = i < len ? data[i] : 0;
        const uint32_t octet_b = i + 1 < len ? data[i + 1] : 0;
        const uint32_t octet_c = i + 2 < len ? data[i + 2] : 0;
        const uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

        out.push_back(kTable[(triple >> 18) & 0x3F]);
        out.push_back(kTable[(triple >> 12) & 0x3F]);
        out.push_back(i + 1 < len ? kTable[(triple >> 6) & 0x3F] : '=');
        out.push_back(i + 2 < len ? kTable[triple & 0x3F] : '=');
    }
    return out;
}
#endif

} // namespace

EventSigner::EventSigner(Config cfg)
    : cfg_(std::move(cfg))
{}

bool EventSigner::loadKey()
{
    hmac_key_.clear();
    if (cfg_.hmac_key_file.empty())
        return false;

    std::ifstream file(cfg_.hmac_key_file, std::ios::binary);
    if (!file.is_open())
        return false;

    std::ostringstream oss;
    oss << file.rdbuf();
    hmac_key_ = oss.str();

    while (!hmac_key_.empty() &&
           (hmac_key_.back() == '\n' || hmac_key_.back() == '\r'))
        hmac_key_.pop_back();

    return !hmac_key_.empty();
}

std::string EventSigner::canonicalPayload(const HazardObservation& obs) const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    const std::string eventId = uuidBytesToString(obs.event_id);
    const std::string observedAt = formatObservedAtIso8601();

    oss << '{'
        << "\"device_id\":\"" << obs.device_id << "\","
        << "\"device_seq\":" << obs.device_seq << ','
        << "\"event_id\":\"" << eventId << "\","
        << "\"hazard\":{\"bbox\":[" << obs.bbox_x << ',' << obs.bbox_y << ','
        << obs.bbox_w << ',' << obs.bbox_h << "],"
        << "\"frame_index\":" << obs.frame_index << ','
        << "\"geometry_conf\":" << obs.geometry_conf << ','
        << "\"iss\":" << obs.iss << ','
        << "\"rri\":" << obs.rri << ','
        << "\"temporal_conf\":" << obs.temporal_conf << ','
        << "\"yolo_conf\":" << obs.yolo_conf
        << "},"
        << "\"hazard_class\":" << static_cast<unsigned>(obs.hazard_class) << ','
        << "\"location\":{\"lat\":" << obs.lat << ",\"lon\":" << obs.lon << "},"
        << "\"motion\":{\"fix_type\":" << static_cast<unsigned>(obs.gps_fix_type)
        << ",\"hdop\":" << obs.hdop << ",\"speed_mps\":" << obs.speed_ms << "},"
        << "\"observed_at\":\"" << observedAt << "\""
        << '}';

    return oss.str();
}

std::string EventSigner::sha256Hex(const std::string& data) const
{
#if defined(VIGIA_HAVE_OPENSSL)
    uint8_t digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;

    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    if (!ctx)
        return {};

    if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1 ||
        EVP_DigestUpdate(ctx, data.data(), data.size()) != 1 ||
        EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
        EVP_MD_CTX_free(ctx);
        return {};
    }
    EVP_MD_CTX_free(ctx);

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < digest_len; ++i)
        oss << std::setw(2) << static_cast<unsigned>(digest[i]);
    return oss.str();
#else
    (void)data;
    return "dev-no-openssl-sha256";
#endif
}

std::string EventSigner::hmacSha256Base64(const std::string& data) const
{
#if defined(VIGIA_HAVE_OPENSSL)
    if (hmac_key_.empty())
        return {};

    uint8_t digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;

    if (!HMAC(EVP_sha256(),
              hmac_key_.data(), static_cast<int>(hmac_key_.size()),
              reinterpret_cast<const uint8_t*>(data.data()), data.size(),
              digest, &digest_len))
        return {};

    return base64Encode(digest, digest_len);
#else
    (void)data;
    return "dev-no-openssl-hmac";
#endif
}

std::string EventSigner::signEnvelope(const HazardObservation& obs) const
{
    const std::string canonical = canonicalPayload(obs);
    const std::string payloadHash = sha256Hex(canonical);
    const std::string signature = hmacSha256Base64(canonical);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);

    const std::string eventId = uuidBytesToString(obs.event_id);
    const std::string observedAt = formatObservedAtIso8601();

    oss << '{'
        << "\"event_id\":\"" << eventId << "\","
        << "\"device_id\":\"" << obs.device_id << "\","
        << "\"device_seq\":" << obs.device_seq << ','
        << "\"observed_at\":\"" << observedAt << "\","
        << "\"hazard_class\":" << static_cast<unsigned>(obs.hazard_class) << ','
        << "\"location\":{\"lat\":" << obs.lat << ",\"lon\":" << obs.lon << "},"
        << "\"hazard\":{"
        << "\"rri\":" << obs.rri << ','
        << "\"iss\":" << obs.iss << ','
        << "\"yolo_conf\":" << obs.yolo_conf << ','
        << "\"geometry_conf\":" << obs.geometry_conf << ','
        << "\"temporal_conf\":" << obs.temporal_conf << ','
        << "\"bbox\":[" << obs.bbox_x << ',' << obs.bbox_y << ','
        << obs.bbox_w << ',' << obs.bbox_h << "],"
        << "\"frame_index\":" << obs.frame_index
        << "},"
        << "\"motion\":{"
        << "\"speed_mps\":" << obs.speed_ms << ','
        << "\"hdop\":" << obs.hdop << ','
        << "\"fix_type\":" << static_cast<unsigned>(obs.gps_fix_type)
        << "},"
        << "\"payload_hash\":\"" << payloadHash << "\","
        << "\"signature\":\"" << signature << "\","
        << "\"signed_et\":null"
        << '}';

    return oss.str();
}

} // namespace vigia
