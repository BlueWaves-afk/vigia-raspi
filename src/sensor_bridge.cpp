#include "sensor_bridge.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

namespace vigia {
namespace {

speed_t baudToTermios(int baud) {
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
    }
}

void trimTrailingCrLf(std::string& line) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
        line.pop_back();
}

} // namespace

SensorBridge::SensorBridge()
    : SensorBridge(Config{}) {}

SensorBridge::SensorBridge(Config config)
    : config_(std::move(config)) {}

SensorBridge::~SensorBridge() {
    stop();
}

void SensorBridge::start() {
    if (running_.exchange(true))
        return;

    if (!openSerial()) {
        running_ = false;
        return;
    }

    reader_thread_ = std::thread(&SensorBridge::readLoop, this);
}

void SensorBridge::stop() {
    if (!running_.exchange(false))
        return;

    closeSerial();

    if (reader_thread_.joinable())
        reader_thread_.join();
}

bool SensorBridge::openSerial() {
    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0)
        return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        closeSerial();
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baudToTermios(config_.baud));
    cfsetospeed(&tty, baudToTermios(config_.baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        closeSerial();
        return false;
    }

    return true;
}

void SensorBridge::closeSerial() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SensorBridge::readLoop() {
    std::string pending;
    pending.reserve(512);

    while (running_.load()) {
        if (fd_ < 0)
            break;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;

        const int ready = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            break;
        }

        if (ready == 0)
            continue;

        char chunk[256];
        const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (n == 0)
            break;

        pending.append(chunk, static_cast<std::size_t>(n));

        std::size_t pos = 0;
        while (true) {
            const std::size_t nl = pending.find('\n', pos);
            if (nl == std::string::npos)
                break;

            std::string line = pending.substr(pos, nl - pos);
            trimTrailingCrLf(line);
            if (!line.empty())
                processLine(line);

            pos = nl + 1;
        }

        if (pos > 0)
            pending.erase(0, pos);
    }
}

void SensorBridge::processLine(const std::string& line) {
    if (auto imu = parseImuLine(line)) {
        handleImu(*imu);
        return;
    }

    if (auto gps = parseGpsLine(line)) {
        handleGps(*gps);
        return;
    }

    if (auto ping = parsePingLine(line)) {
        handlePing(*ping);
        return;
    }

    if (line.rfind("VIGIA_", 0) == 0)
        recordParseError();
}

void SensorBridge::handleImu(const ImuSample& sample) {
    ++health_.imu_count;

    if (health_.have_imu_seq) {
        const uint32_t expected = health_.last_imu_seq + 1;
        if (sample.seq != expected && sample.seq > health_.last_imu_seq)
            health_.imu_seq_gaps += sample.seq - expected;
    }

    health_.last_imu_seq = sample.seq;
    health_.have_imu_seq = true;

    state_.updateImu(sample);
    state_.updateHealth(health_);
}

void SensorBridge::handleGps(const GpsFix& fix) {
    ++health_.gps_count;

    if (health_.have_gps_seq) {
        const uint32_t expected = health_.last_gps_seq + 1;
        if (fix.seq != expected && fix.seq > health_.last_gps_seq)
            health_.gps_seq_gaps += fix.seq - expected;
    }

    health_.last_gps_seq = fix.seq;
    health_.have_gps_seq = true;

    state_.updateGps(fix);
    state_.updateHealth(health_);
}

void SensorBridge::handlePing(const PingReport& ping) {
    ++health_.ping_count;
    health_.last_ping_uptime_ms = ping.uptime_ms;
    state_.updateHealth(health_);
}

void SensorBridge::recordParseError() {
    ++health_.parse_errors;
    state_.updateHealth(health_);
}

} // namespace vigia
