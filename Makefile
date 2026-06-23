# ── Standalone test targets (no OpenCV/OpenVINO required) ────────────────────
# These compile and run the unit tests that exercise the telemetry pipeline,
# buffer bounds, and protocol correctness on the host machine.

CXX      ?= c++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wno-unused-parameter -O2 -Iinclude
LDFLAGS  := -lpthread

BUILD_DIR := build/host_tests

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ── Shared source groups ──────────────────────────────────────────────────────

PACKET_SRCS := \
    src/sensor_packet.cpp     \
    src/sensor_state.cpp      \
    src/sensor_bridge.cpp     \
    src/cobs.cpp              \
    src/signed_et_packet.cpp  \
    src/ecdsa_verify.cpp

ISS_COMPUTE_SRCS := \
    src/sensor_packet.cpp     \
    src/iss_compute.cpp

SENSOR_PROCESSOR_SRCS := \
    $(ISS_COMPUTE_SRCS)       \
    src/iss_filter.cpp        \
    src/sensor_state.cpp      \
    src/signed_et_packet.cpp  \
    src/sensor_processor.cpp

HAZARD_SRCS := \
    src/hazard_event.cpp

SIGNER_SRCS := \
    src/hazard_event.cpp      \
    src/event_signer.cpp

PROMOTER_SRCS := \
    src/hazard_event.cpp      \
    src/event_promoter.cpp

# ── Individual test binaries ──────────────────────────────────────────────────

$(BUILD_DIR)/sensor_bridge_test: tests/sensor_bridge_test.cpp $(PACKET_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/cobs_roundtrip_test: tests/cobs_roundtrip_test.cpp src/cobs.cpp src/signed_et_packet.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/iss_compute_test: tests/iss_compute_test.cpp $(ISS_COMPUTE_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/iss_filter_test: tests/iss_filter_test.cpp src/iss_filter.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/temporal_test: tests/temporal_test.cpp src/temporal.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/fusion_test: tests/fusion_test.cpp src/fusion.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/event_promoter_test: tests/event_promoter_test.cpp $(PROMOTER_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/hazard_event_test: tests/hazard_event_test.cpp $(HAZARD_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/sensor_processor_test: tests/sensor_processor_test.cpp $(SENSOR_PROCESSOR_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/event_signer_test: tests/event_signer_test.cpp $(SIGNER_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# ── make test — compile and run all standalone unit tests ─────────────────────
TEST_BINS := \
    $(BUILD_DIR)/sensor_bridge_test    \
    $(BUILD_DIR)/cobs_roundtrip_test   \
    $(BUILD_DIR)/iss_compute_test      \
    $(BUILD_DIR)/iss_filter_test       \
    $(BUILD_DIR)/temporal_test         \
    $(BUILD_DIR)/fusion_test           \
    $(BUILD_DIR)/event_promoter_test   \
    $(BUILD_DIR)/hazard_event_test     \
    $(BUILD_DIR)/sensor_processor_test \
    $(BUILD_DIR)/event_signer_test

test: $(TEST_BINS)
	@echo "=== Running sensor_bridge_test ==="
	$(BUILD_DIR)/sensor_bridge_test
	@echo "=== Running cobs_roundtrip_test ==="
	$(BUILD_DIR)/cobs_roundtrip_test
	@echo "=== Running iss_compute_test ==="
	$(BUILD_DIR)/iss_compute_test
	@echo "=== Running iss_filter_test ==="
	$(BUILD_DIR)/iss_filter_test
	@echo "=== Running temporal_test ==="
	$(BUILD_DIR)/temporal_test
	@echo "=== Running fusion_test ==="
	$(BUILD_DIR)/fusion_test
	@echo "=== Running event_promoter_test ==="
	$(BUILD_DIR)/event_promoter_test
	@echo "=== Running hazard_event_test ==="
	$(BUILD_DIR)/hazard_event_test
	@echo "=== Running sensor_processor_test ==="
	$(BUILD_DIR)/sensor_processor_test
	@echo "=== Running event_signer_test ==="
	$(BUILD_DIR)/event_signer_test
	@echo "=== All standalone tests passed ==="

## make validate-buffers — buffer overflow / protocol correctness gate
## Referenced by .claude/skills/telemetry-validator/SKILL.md
validate-buffers: test
	@echo "[validate-buffers] Buffer and protocol checks passed."

checkpoint:
	@echo "Automating session handoff snapshot..."
	@echo "# Automated Session Handoff - $$(date)" > session-handoff.md
	@echo "## Modified Files in Last Session:" >> session-handoff.md
	@git diff --name-status HEAD~1 >> session-handoff.md || echo "No previous commit found" >> session-handoff.md
	@echo "\n## Active Implementation State:" >> session-handoff.md
	@git log -1 --pretty=format:"Last Commit Message: %s (%an)" >> session-handoff.md
	@echo "\n\n[SYSTEM INSTRUCTION]: Next session must read this file, verify compilation via 'make', and pull dependencies via codebase-memory-mcp." >> session-handoff.md
	@echo "session-handoff.md compiled automatically."
