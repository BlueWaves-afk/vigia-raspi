/**
 * sensor_fusion_live_test — bench / field diagnostic for the full ISS pipeline.
 *
 * Runs SensorBridge + SensorProcessor and prints filter diagnostics every
 * second.  Use this at home with the car to validate bumps without needing
 * YOLO to fire first.
 *
 *   ./sensor_fusion_live_test --port /dev/ttyACM0 --duration 60
 *   ./sensor_fusion_live_test --port /dev/ttyACM0 --duration 300 --log run.csv
 */

#include "sensor_bridge.hpp"
#include "sensor_processor.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

using namespace vigia;

namespace {

void usage(const char* argv0)
{
    std::cout
        << "Usage: " << argv0
        << " [--port /dev/ttyACM0] [--duration SEC] [--log FILE.csv]\n\n"
        << "Prints once per second:\n"
        << "  raw awz, normalized ISS, EMA smoothed, background, detrended,\n"
        << "  threshold, genuine-impact flag, GPS speed/lat/lon\n\n"
        << "Stop pico_*_monitor.py before running — only one process may hold the port.\n";
}

void printSnapshot(const SensorProcessor::Snapshot& s, const SensorHealth& h)
{
    std::cout << std::fixed << std::setprecision(4)
              << "  imu_count=" << h.imu_count
              << " gps_count=" << h.gps_count
              << " parse_err=" << h.parse_errors;

    if (!s.imuValid) {
        std::cout << " | IMU: (none)\n";
        return;
    }

    std::cout << "\n  raw_awz=" << s.rawVerticalMs2 << " m/s²"
              << "  norm_iss=" << s.normalizedIss
              << "  smoothed=" << s.filter.smoothedIss
              << "  background=" << s.filter.backgroundIss
              << "  detrended=" << s.filter.detrendedIss
              << "  threshold=" << s.filter.effectiveThreshold
              << "  IMPACT=" << (s.filter.isGenuineImpact ? "YES" : "no")
              << "  fusion_iss=" << s.imuIss;

    if (s.gpsValid) {
        std::cout << std::setprecision(6)
                  << "\n  GPS: lat=" << s.gpsLat
                  << " lon=" << s.gpsLon
                  << std::setprecision(2)
                  << " spd=" << s.speedMs << " m/s";
    } else {
        std::cout << "\n  GPS: (no valid fix)";
    }
    std::cout << '\n';
}

void writeCsvHeader(std::ofstream& out)
{
    out << "t_sec,raw_awz,norm_iss,smoothed,background,detrended,threshold,"
           "impact,fusion_iss,speed_ms,gps_valid,lat,lon,imu_count,gps_count\n";
}

void writeCsvRow(std::ofstream& out, int t_sec, const SensorProcessor::Snapshot& s,
                 const SensorHealth& h)
{
    out << t_sec << ','
        << s.rawVerticalMs2 << ','
        << s.normalizedIss << ','
        << s.filter.smoothedIss << ','
        << s.filter.backgroundIss << ','
        << s.filter.detrendedIss << ','
        << s.filter.effectiveThreshold << ','
        << (s.filter.isGenuineImpact ? 1 : 0) << ','
        << s.imuIss << ','
        << s.speedMs << ','
        << (s.gpsValid ? 1 : 0) << ','
        << s.gpsLat << ','
        << s.gpsLon << ','
        << h.imu_count << ','
        << h.gps_count << '\n';
}

} // namespace

int main(int argc, char** argv)
{
    std::string port = "/dev/ttyACM0";
    int duration_sec = 30;
    std::string logPath;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return 0;
        }
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--log" && i + 1 < argc) {
            logPath = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            usage(argv[0]);
            return 1;
        }
    }

    std::cout << "[FUSION-LIVE] port=" << port
              << " duration=" << duration_sec << "s";
    if (!logPath.empty())
        std::cout << " log=" << logPath;
    std::cout << '\n';

    SensorBridge bridge(SensorBridge::Config{port, 115200});
    SensorProcessor processor;

    bridge.start();
    if (!bridge.isRunning()) {
        std::cerr << "[FUSION-LIVE] Failed to open " << port << '\n';
        return 1;
    }

    std::ofstream csv;
    if (!logPath.empty()) {
        csv.open(logPath);
        if (!csv.is_open()) {
            std::cerr << "[FUSION-LIVE] Cannot open log: " << logPath << '\n';
            bridge.stop();
            return 1;
        }
        writeCsvHeader(csv);
        std::cout << "[FUSION-LIVE] Logging to " << logPath << '\n';
    }

    std::cout << "[FUSION-LIVE] Tap/shake the mount or drive — watch IMPACT=YES on bumps.\n\n";

    for (int sec = 1; sec <= duration_sec; ++sec) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto snap = processor.process(bridge.state());
        const auto health = bridge.state().getHealth();

        std::cout << "[FUSION-LIVE] t=" << sec << "s";
        printSnapshot(snap, health);

        if (csv.is_open())
            writeCsvRow(csv, sec, snap, health);
    }

    bridge.stop();
    std::cout << "[FUSION-LIVE] Done.\n";
    return 0;
}
