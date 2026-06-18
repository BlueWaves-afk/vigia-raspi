#pragma once

#include "sensor_state.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace vigia {

class EcdsaVerifier;

class SensorBridge {
public:
    enum class WireProto { Unknown, Text, Cobs };

    struct Config {
        std::string device{"/dev/ttyACM0"};
        int baud{115200};
        std::string pubkey_file;
        bool allow_stub_sig{false};
        /** Max bytes accumulated in a single COBS frame before it is discarded. */
        std::size_t max_cobs_frame_bytes{512};
        /** Max bytes buffered in the text line accumulator before it is flushed. */
        std::size_t max_pending_bytes{4096};
        /** Delay between reconnect attempts after a hardware disconnect. */
        std::chrono::milliseconds reconnect_delay_ms{2000};
    };

    explicit SensorBridge();
    explicit SensorBridge(Config config);
    ~SensorBridge();

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    void start();
    void stop();

    bool isRunning() const { return running_.load(); }
    WireProto wireProto() const { return proto_; }

    const SensorState& state() const { return state_; }
    SensorState& state() { return state_; }

    void processLine(const std::string& line);
    void processCobsFrame(const std::uint8_t* src, std::size_t src_len);

private:
    void readLoop();
    bool openSerial();
    void closeSerial();
    void handleImu(const ImuSample& sample);
    void handleGps(const GpsFix& fix);
    void handlePing(const PingReport& ping);
    void handleSignedEt(const SignedEtSample& sample);
    void recordParseError();
    void detectProto(std::uint8_t byte);

    Config config_;
    SensorState state_;
    SensorHealth health_{};
    WireProto proto_{WireProto::Unknown};

    std::vector<std::uint8_t> cobs_acc_;
    bool in_cobs_frame_{false};

    std::unique_ptr<EcdsaVerifier> verifier_;

    std::thread reader_thread_;
    std::atomic<bool> running_{false};
    int fd_{-1};
};

} // namespace vigia
