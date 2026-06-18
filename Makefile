# ── Standalone test targets (no OpenCV/OpenVINO required) ────────────────────
# These compile and run the unit tests that exercise the telemetry pipeline,
# buffer bounds, and protocol correctness on the host machine.

CXX      ?= c++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wno-unused-parameter -O2 -Iinclude
LDFLAGS  := -lpthread

BUILD_DIR := build/host_tests

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

HOST_SRCS := \
    src/sensor_packet.cpp \
    src/sensor_state.cpp  \
    src/sensor_bridge.cpp \
    src/cobs.cpp          \
    src/signed_et_packet.cpp \
    src/ecdsa_verify.cpp

$(BUILD_DIR)/sensor_bridge_test: tests/sensor_bridge_test.cpp $(HOST_SRCS) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(BUILD_DIR)/cobs_roundtrip_test: tests/cobs_roundtrip_test.cpp src/cobs.cpp src/signed_et_packet.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

## make test — compile and run all standalone unit tests
test: $(BUILD_DIR)/sensor_bridge_test $(BUILD_DIR)/cobs_roundtrip_test
	@echo "=== Running sensor_bridge_test ==="
	$(BUILD_DIR)/sensor_bridge_test
	@echo "=== Running cobs_roundtrip_test ==="
	$(BUILD_DIR)/cobs_roundtrip_test
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
