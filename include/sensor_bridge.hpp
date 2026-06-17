#pragma once

#include "sensor_state.hpp"

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace vigia {

class SensorBridge {
public:
    struct Config {
        std::string device{"/dev/ttyACM0"};
        int baud{115200};
    };

    explicit SensorBridge();
    explicit SensorBridge(Config config);
    ~SensorBridge();

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    void start();
    void stop();

    bool isRunning() const { return running_.load(); }

    const SensorState& state() const { return state_; }
    SensorState& state() { return state_; }

    void processLine(const std::string& line);

private:
    void readLoop();
    bool openSerial();
    void closeSerial();
    void handleImu(const ImuSample& sample);
    void handleGps(const GpsFix& fix);
    void handlePing(const PingReport& ping);
    void recordParseError();

    Config config_;
    SensorState state_;
    SensorHealth health_{};

    std::thread reader_thread_;
    std::atomic<bool> running_{false};
    int fd_{-1};
};

} // namespace vigia
