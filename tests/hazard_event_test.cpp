/*
 * hazard_event_test.cpp — unit tests for hazard_event serialization helpers.
 *
 * Covers:
 *   - uuidBytesToString: layout, hyphen placement, hex case
 *   - hazardObservationToJson: required JSON keys, value round-trip
 *
 * Build note: compiled via the Makefile `test` target.
 */

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

/** Returns true if `haystack` contains `needle` as a substring. */
bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

HazardObservation makeObs() {
    HazardObservation obs{};

    // Stable UUID bytes for deterministic string output.
    const uint8_t id[16] = {
        0x01, 0x92, 0xAB, 0xCD, 0x12, 0x34, 0x56, 0x78,
        0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67
    };
    std::memcpy(obs.event_id, id, 16);

    std::strncpy(obs.device_id, "vigia-test-001", sizeof(obs.device_id) - 1);
    obs.device_seq      = 42;
    obs.frame_index     = 100;
    obs.hazard_class    = static_cast<uint8_t>(HazardClass::Pothole);
    obs.rri             = 0.82f;
    obs.iss             = 0.45f;
    obs.yolo_conf       = 0.91f;
    obs.geometry_conf   = 0.72f;
    obs.temporal_conf   = 0.61f;
    obs.bbox_x          = 120;
    obs.bbox_y          = 200;
    obs.bbox_w          = 80;
    obs.bbox_h          = 60;
    obs.lat             = 12.971600;
    obs.lon             = 77.594600;
    obs.speed_ms        = 8.3f;
    obs.hdop            = 1.2f;
    obs.gps_fix_type    = 3;
    obs.gps_valid       = true;
    obs.signed_et_valid = false;
    return obs;
}

} // namespace

/* ─────────────────────────────────────────────────────────────────────────
 * Section 1 — uuidBytesToString
 * ───────────────────────────────────────────────────────────────────────── */
static void test_uuid_format() {
    std::cout << "\n[TEST] uuidBytesToString — format and layout\n";

    const uint8_t id[16] = {
        0x01, 0x92, 0xAB, 0xCD, 0x12, 0x34, 0x56, 0x78,
        0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67
    };

    const std::string s = uuidBytesToString(id);

    // Length: 8-4-4-4-12 = 32 hex + 4 hyphens = 36 chars.
    expectTrue(s.size() == 36, "UUID string is 36 characters");

    // Hyphen positions: indices 8, 13, 18, 23.
    expectTrue(s[8]  == '-', "hyphen at index 8");
    expectTrue(s[13] == '-', "hyphen at index 13");
    expectTrue(s[18] == '-', "hyphen at index 18");
    expectTrue(s[23] == '-', "hyphen at index 23");

    // Expected value from the known input bytes above.
    expectTrue(s == "0192abcd-1234-5678-89ab-cdef01234567",
               "UUID string matches expected hex");
}

static void test_uuid_all_zeros() {
    std::cout << "\n[TEST] uuidBytesToString — all-zero bytes\n";
    const uint8_t id[16] = {};
    const std::string s = uuidBytesToString(id);
    expectTrue(s == "00000000-0000-0000-0000-000000000000",
               "all-zero UUID is canonical nil UUID");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Section 2 — hazardObservationToJson
 * ───────────────────────────────────────────────────────────────────────── */
static void test_json_required_keys() {
    std::cout << "\n[TEST] hazardObservationToJson — required keys present\n";

    const auto obs = makeObs();
    const std::string json = hazardObservationToJson(obs);

    expectTrue(!json.empty(), "JSON output is non-empty");
    expectTrue(json.front() == '{' && json.back() == '}',
               "output is wrapped in braces");

    expectTrue(contains(json, "\"event_id\""),    "contains event_id");
    expectTrue(contains(json, "\"device_id\""),   "contains device_id");
    expectTrue(contains(json, "\"device_seq\""),  "contains device_seq");
    expectTrue(contains(json, "\"observed_at\""), "contains observed_at");
    expectTrue(contains(json, "\"hazard_class\""),"contains hazard_class");
    expectTrue(contains(json, "\"location\""),    "contains location");
    expectTrue(contains(json, "\"hazard\""),      "contains hazard");
    expectTrue(contains(json, "\"motion\""),      "contains motion");
}

static void test_json_values() {
    std::cout << "\n[TEST] hazardObservationToJson — key values\n";

    const auto obs = makeObs();
    const std::string json = hazardObservationToJson(obs);

    // device_id must appear literally.
    expectTrue(contains(json, "vigia-test-001"), "device_id value in JSON");

    // device_seq = 42.
    expectTrue(contains(json, "\"device_seq\":42"), "device_seq value = 42");

    // hazard_class = 0 (Pothole).
    expectTrue(contains(json, "\"hazard_class\":0"), "hazard_class value = 0");

    // bbox values.
    expectTrue(contains(json, "120"), "bbox_x present");
    expectTrue(contains(json, "200"), "bbox_y present");
    expectTrue(contains(json, "\"frame_index\":100"), "frame_index present");
}

static void test_json_event_id_matches_uuid() {
    std::cout << "\n[TEST] hazardObservationToJson — event_id matches uuidBytesToString\n";

    const auto obs = makeObs();
    const std::string json   = hazardObservationToJson(obs);
    const std::string uuid_s = uuidBytesToString(obs.event_id);

    expectTrue(contains(json, uuid_s), "event_id in JSON equals uuidBytesToString output");
}

static void test_json_device_id_null_safety() {
    std::cout << "\n[TEST] hazardObservationToJson — non-null-terminated device_id\n";

    HazardObservation obs = makeObs();
    // Fill entire device_id buffer with 'X' (no null terminator).
    std::memset(obs.device_id, 'X', sizeof(obs.device_id));

    // Must not crash or produce undefined output.
    const std::string json = hazardObservationToJson(obs);
    expectTrue(!json.empty(), "JSON produced for non-null-terminated device_id");
    // The output must still be braced JSON.
    expectTrue(json.front() == '{' && json.back() == '}',
               "output remains valid JSON object with no null terminator");
}

int main() {
    std::cout << "[TEST] ===== HazardEvent Serialization Unit Test =====\n";

    test_uuid_format();
    test_uuid_all_zeros();
    test_json_required_keys();
    test_json_values();
    test_json_event_id_matches_uuid();
    test_json_device_id_null_safety();

    std::cout << '\n';
    if (g_failed == 0) {
        std::cout << "[TEST] All hazard_event tests passed.\n";
        return 0;
    }
    std::cerr << "[TEST] " << g_failed << " test(s) failed.\n";
    return 1;
}
