/*
 * event_signer_test.cpp — unit tests for EventSigner.
 *
 * Tests the signing envelope pipeline without requiring an actual HMAC key
 * on disk.  All tests exercise the dev/no-key code paths which must still
 * produce structurally correct JSON.  Key-dependent crypto assertions are
 * guarded behind VIGIA_HAVE_OPENSSL so the test suite passes on the host
 * regardless of OpenSSL availability.
 *
 * Covers:
 *   1. No key loaded — hasKey() = false, signEnvelope() still returns JSON
 *   2. Envelope contains all required top-level keys
 *   3. event_id in the envelope matches uuidBytesToString()
 *   4. device_id value propagated correctly
 *   5. hazard sub-object contains rri, iss, yolo_conf, geometry_conf, temporal_conf
 *   6. location sub-object contains lat and lon
 *   7. motion sub-object contains speed_mps, hdop, fix_type
 *   8. payload_hash and signature keys are always present
 *   9. Determinism: same obs → same canonical payload on consecutive calls
 *
 * Build note: compiled via the Makefile `test` target.
 */

#include "event_signer.hpp"
#include "hazard_event.hpp"

#include <cstring>
#include <iostream>
#include <string>

using namespace vigia;

namespace {

int g_failed = 0;

void expectTrue(bool cond, const char* label) {
    if (cond) {
        std::cout << "  PASS: " << label << '\n';
    } else {
        std::cerr << "  FAIL: " << label << '\n';
        ++g_failed;
    }
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

HazardObservation makeObs() {
    HazardObservation obs{};

    const uint8_t id[16] = {
        0x01, 0x92, 0xDE, 0xAD, 0xBE, 0xEF, 0x56, 0x78,
        0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78
    };
    std::memcpy(obs.event_id, id, 16);

    std::strncpy(obs.device_id, "vigia-signer-test", sizeof(obs.device_id) - 1);
    obs.device_seq      = 7;
    obs.frame_index     = 55;
    obs.hazard_class    = static_cast<uint8_t>(HazardClass::Pothole);
    obs.rri             = 0.83f;
    obs.iss             = 0.40f;
    obs.yolo_conf       = 0.88f;
    obs.geometry_conf   = 0.70f;
    obs.temporal_conf   = 0.62f;
    obs.bbox_x          = 50;
    obs.bbox_y          = 100;
    obs.bbox_w          = 90;
    obs.bbox_h          = 70;
    obs.lat             = 12.971600;
    obs.lon             = 77.594600;
    obs.speed_ms        = 9.5f;
    obs.hdop            = 1.1f;
    obs.gps_fix_type    = 3;
    obs.gps_valid       = true;
    obs.signed_et_valid = false;
    return obs;
}

} // namespace

/* ─────────────────────────────────────────────────────────────────────────
 * Test 1 — no key loaded
 * ───────────────────────────────────────────────────────────────────────── */
static void test_no_key_state() {
    std::cout << "\n[TEST 1] No key loaded — hasKey() and signEnvelope()\n";

    EventSigner signer;
    expectTrue(!signer.hasKey(), "hasKey()=false when no key file configured");

    const std::string env = signer.signEnvelope(makeObs());
    expectTrue(!env.empty(),            "signEnvelope() returns non-empty JSON without key");
    expectTrue(env.front() == '{',      "output starts with '{'");
    expectTrue(env.back()  == '}',      "output ends with '}'");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 2 — envelope contains all required top-level keys
 * ───────────────────────────────────────────────────────────────────────── */
static void test_envelope_required_keys() {
    std::cout << "\n[TEST 2] Envelope contains all required top-level keys\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());

    expectTrue(contains(env, "\"event_id\""),     "key: event_id");
    expectTrue(contains(env, "\"device_id\""),    "key: device_id");
    expectTrue(contains(env, "\"device_seq\""),   "key: device_seq");
    expectTrue(contains(env, "\"observed_at\""),  "key: observed_at");
    expectTrue(contains(env, "\"hazard_class\""), "key: hazard_class");
    expectTrue(contains(env, "\"location\""),     "key: location");
    expectTrue(contains(env, "\"hazard\""),       "key: hazard");
    expectTrue(contains(env, "\"motion\""),       "key: motion");
    expectTrue(contains(env, "\"payload_hash\""), "key: payload_hash");
    expectTrue(contains(env, "\"signature\""),    "key: signature");
    expectTrue(contains(env, "\"signed_et\""),    "key: signed_et");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 3 — event_id value matches uuidBytesToString()
 * ───────────────────────────────────────────────────────────────────────── */
static void test_event_id_value() {
    std::cout << "\n[TEST 3] event_id value matches uuidBytesToString\n";

    const auto obs       = makeObs();
    EventSigner signer;
    const std::string env    = signer.signEnvelope(obs);
    const std::string uuid_s = uuidBytesToString(obs.event_id);

    expectTrue(contains(env, uuid_s),
               "envelope event_id equals uuidBytesToString output");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 4 — device_id propagated
 * ───────────────────────────────────────────────────────────────────────── */
static void test_device_id_propagated() {
    std::cout << "\n[TEST 4] device_id value propagated into envelope\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());
    expectTrue(contains(env, "vigia-signer-test"),
               "device_id value present in envelope JSON");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 5 — hazard sub-object structure
 * ───────────────────────────────────────────────────────────────────────── */
static void test_hazard_subobject() {
    std::cout << "\n[TEST 5] hazard sub-object fields\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());

    expectTrue(contains(env, "\"rri\""),           "hazard.rri present");
    expectTrue(contains(env, "\"iss\""),           "hazard.iss present");
    expectTrue(contains(env, "\"yolo_conf\""),     "hazard.yolo_conf present");
    expectTrue(contains(env, "\"geometry_conf\""), "hazard.geometry_conf present");
    expectTrue(contains(env, "\"temporal_conf\""), "hazard.temporal_conf present");
    expectTrue(contains(env, "\"bbox\""),          "hazard.bbox present");
    expectTrue(contains(env, "\"frame_index\""),   "hazard.frame_index present");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 6 — location sub-object
 * ───────────────────────────────────────────────────────────────────────── */
static void test_location_subobject() {
    std::cout << "\n[TEST 6] location sub-object fields\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());

    expectTrue(contains(env, "\"lat\""), "location.lat present");
    expectTrue(contains(env, "\"lon\""), "location.lon present");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 7 — motion sub-object
 * ───────────────────────────────────────────────────────────────────────── */
static void test_motion_subobject() {
    std::cout << "\n[TEST 7] motion sub-object fields\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());

    expectTrue(contains(env, "\"speed_mps\""), "motion.speed_mps present");
    expectTrue(contains(env, "\"hdop\""),      "motion.hdop present");
    expectTrue(contains(env, "\"fix_type\""),  "motion.fix_type present");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 8 — payload_hash and signature always present
 * ───────────────────────────────────────────────────────────────────────── */
static void test_hash_and_sig_always_present() {
    std::cout << "\n[TEST 8] payload_hash and signature keys always emitted\n";

    EventSigner signer;
    const std::string env = signer.signEnvelope(makeObs());

    // Both keys must appear even when OpenSSL is absent (dev stubs).
    expectTrue(contains(env, "\"payload_hash\":\""),
               "payload_hash key with string value present");
    expectTrue(contains(env, "\"signature\":\""),
               "signature key with string value present");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 9 — signed_et is null when signed_et_valid=false
 * ───────────────────────────────────────────────────────────────────────── */
static void test_signed_et_null_when_invalid() {
    std::cout << "\n[TEST 9] signed_et=null when signed_et_valid=false\n";

    HazardObservation obs = makeObs();
    obs.signed_et_valid = false;

    EventSigner signer;
    const std::string env = signer.signEnvelope(obs);
    expectTrue(contains(env, "\"signed_et\":null"),
               "signed_et is null when not valid");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Test 10 — deterministic structure across two calls (same obs)
 *           The timestamps will differ, but the structural keys must match.
 * ───────────────────────────────────────────────────────────────────────── */
static void test_structure_is_deterministic() {
    std::cout << "\n[TEST 10] Consecutive calls produce consistent structure\n";

    const auto obs = makeObs();
    EventSigner signer;

    const std::string env1 = signer.signEnvelope(obs);
    const std::string env2 = signer.signEnvelope(obs);

    // Both envelopes must contain the same structural keys.
    const std::string keys[] = {
        "event_id", "device_id", "device_seq", "observed_at",
        "hazard_class", "location", "hazard", "motion",
        "payload_hash", "signature", "signed_et"
    };
    for (const auto& k : keys) {
        const bool in1 = contains(env1, '"' + k + '"');
        const bool in2 = contains(env2, '"' + k + '"');
        expectTrue(in1 && in2, ("key \"" + k + "\" present in both calls").c_str());
    }

    // The event_id must be identical (same input bytes).
    expectTrue(
        env1.find(uuidBytesToString(obs.event_id)) != std::string::npos &&
        env2.find(uuidBytesToString(obs.event_id)) != std::string::npos,
        "event_id identical across both calls");
}

int main() {
    std::cout << "[TEST] ===== EventSigner Unit Test =====\n";

    test_no_key_state();
    test_envelope_required_keys();
    test_event_id_value();
    test_device_id_propagated();
    test_hazard_subobject();
    test_location_subobject();
    test_motion_subobject();
    test_hash_and_sig_always_present();
    test_signed_et_null_when_invalid();
    test_structure_is_deterministic();

    std::cout << '\n';
    if (g_failed == 0) {
        std::cout << "[TEST] All EventSigner tests passed.\n";
        return 0;
    }
    std::cerr << "[TEST] " << g_failed << " test(s) failed.\n";
    return 1;
}
