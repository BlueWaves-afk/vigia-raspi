#include "sensor_bridge.hpp"

#include "cobs.hpp"
#include "ecdsa_verify.hpp"
#include "signed_et_packet.hpp"

#include <cerrno>
#include <cstring>
#include <memory>
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
    : config_(std::move(config))
{
    cobs_acc_.reserve(256);

    if (!config_.pubkey_file.empty() || config_.allow_stub_sig) {
        EcdsaVerifier::Config vcfg;
        vcfg.pubkey_file = config_.pubkey_file;
        vcfg.allow_stub_sig = config_.allow_stub_sig;
        verifier_ = std::make_unique<EcdsaVerifier>(vcfg);
        verifier_->loadPublicKey();
    }
}

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

void SensorBridge::detectProto(std::uint8_t byte) {
    if (proto_ != WireProto::Unknown)
        return;

    if (byte == 0x56) {
        proto_ = WireProto::Text;
    } else if (byte == 0x00) {
        proto_ = WireProto::Cobs;
        in_cobs_frame_ = false;
        cobs_acc_.clear();
    }
}

void SensorBridge::readLoop() {
    std::string pending;
    pending.reserve(512);

    while (running_.load()) {
        // Attempt (re-)open if the fd is gone.
        if (fd_ < 0) {
            std::this_thread::sleep_for(config_.reconnect_delay_ms);
            if (!running_.load())
                break;
            if (!openSerial()) {
                // Increment parse_errors so callers can detect the outage.
                recordParseError();
                continue;
            }
            // Reset protocol detector and accumulators on reconnect.
            proto_ = WireProto::Unknown;
            cobs_acc_.clear();
            in_cobs_frame_ = false;
            pending.clear();
        }

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
            closeSerial();
            continue;
        }

        if (ready == 0)
            continue;

        std::uint8_t chunk[256];
        const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            // EIO / ENODEV / other hard error — device disconnected.
            closeSerial();
            continue;
        }
        if (n == 0) {
            // EOF — device was unplugged or closed its end.
            closeSerial();
            continue;
        }

        for (ssize_t i = 0; i < n; ++i) {
            const std::uint8_t byte = chunk[i];

            if (proto_ == WireProto::Unknown)
                detectProto(byte);

            if (proto_ == WireProto::Cobs) {
                if (byte == 0x00) {
                    if (!cobs_acc_.empty())
                        processCobsFrame(cobs_acc_.data(), cobs_acc_.size());
                    cobs_acc_.clear();
                    in_cobs_frame_ = false;
                    continue;
                }

                if (!in_cobs_frame_) {
                    in_cobs_frame_ = true;
                    cobs_acc_.clear();
                }

                // Guard against a runaway frame that never terminates.
                if (cobs_acc_.size() >= config_.max_cobs_frame_bytes) {
                    recordParseError();
                    cobs_acc_.clear();
                    in_cobs_frame_ = false;
                    continue;
                }

                cobs_acc_.push_back(byte);
                continue;
            }

            if (proto_ == WireProto::Text || proto_ == WireProto::Unknown) {
                // Guard against a line stream with no newlines.
                if (pending.size() >= config_.max_pending_bytes) {
                    recordParseError();
                    pending.clear();
                }
                pending.push_back(static_cast<char>(byte));
            }
        }

        if (proto_ == WireProto::Text || proto_ == WireProto::Unknown) {
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
}

void SensorBridge::processCobsFrame(const std::uint8_t* src, std::size_t src_len) {
    std::uint8_t decoded[256];
    const std::size_t dec_len = cobsDecode(src, src_len, decoded, sizeof(decoded));
    if (dec_len != kSignedEtPacketSize) {
        recordParseError();
        return;
    }

    SignedEtSample sample{};
    if (!parseSignedEtPacket(decoded, dec_len, sample)) {
        recordParseError();
        return;
    }

    if (verifier_) {
        sample.sig_valid = verifier_->verify(sample.et_hash.data(),
                                             sample.ecdsa_sig.data());
    }

    handleSignedEt(sample);
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

void SensorBridge::handleSignedEt(const SignedEtSample& sample) {
    ++health_.signed_et_count;
    if (sample.sig_valid)
        ++health_.signed_et_valid_count;

    if (health_.have_signed_et_seq) {
        const uint32_t expected = health_.last_signed_et_seq + 1;
        // Only count a gap when sequence advances forward; ignore wrap-around.
        if (sample.sequence != expected && sample.sequence > health_.last_signed_et_seq)
            health_.signed_et_seq_gaps += sample.sequence - expected;
    }

    health_.last_signed_et_seq = sample.sequence;
    health_.have_signed_et_seq = true;

    ImuSample imu{};
    imu.seq = sample.sequence;
    imu.timestamp_us = sample.timestamp_us;
    imu.qw = sample.qw;
    imu.qx = sample.qx;
    imu.qy = sample.qy;
    imu.qz = sample.qz;
    imu.ax = sample.ax;
    imu.ay = sample.ay;
    imu.az = sample.az;
    imu.cal_status = sample.cal_status;
    imu.valid = true;
    state_.updateImu(imu);

    GpsFix gps{};
    gps.seq = sample.sequence;
    gps.timestamp_us = sample.timestamp_us;
    gps.latitude = sample.latitude;
    gps.longitude = sample.longitude;
    gps.speed_ms = sample.speed_ms;
    gps.fix_type = sample.fix_type;
    gps.satellites = sample.satellites;
    gps.valid = sample.fix_type >= 2;
    gps.source = "cobs";
    state_.updateGps(gps);

    state_.updateSignedEt(sample);
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
