#pragma once
// frame_metadata_ring.hpp — Per-frame metadata sidecar ring on /dev/shm.
//
// Sits alongside ShmRingBuffer (pixel frames, 829 MB) but contains only
// compact sensor metadata (36 KB). AntiDeathNode snapshots this during the
// emergency window instead of the raw pixel buffer.
//
// Seqlock protocol (same as ShmRingBuffer):
//   seq even  → ring is stable (reader may snapshot)
//   seq odd   → writer is mid-write (reader must retry)
//
// Shared by:  FusionNode  (writer, SCHED_FIFO 75)
//             AntiDeathNode (snapshot reader, SCHED_FIFO 99)
//
// /dev/shm path: /vigia_meta_ring  (shm_open name, no leading /)

#include <atomic>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static constexpr uint32_t kMetaRingDepth = 300;

// Per-frame metadata — packed to guarantee layout across compiler versions.
// Fields written by FusionNode after each RRI computation cycle.
struct __attribute__((packed)) FrameMetadata {
    uint64_t  timestamp_us;           // CameraNode capture time (steady_clock epoch)
    uint32_t  frame_id;               // Monotonic frame counter

    float     rri_score;              // Road Risk Index [0,1]; -1.0 if not fused yet
    float     iss_score;              // Impact Severity Score; -1.0 if not computed

    // IMU — BNO085 quaternion + gravity-compensated linear accel (body frame)
    float     q_w, q_x, q_y, q_z;
    float     lin_accel_x, lin_accel_y, lin_accel_z;
    uint8_t   imu_cal_status;
    uint8_t   _imu_pad[3];            // explicit zero padding

    // GPS — NEO-M8N output
    double    latitude, longitude;    // degrees, WGS84
    float     altitude_m;
    float     speed_ms;
    float     course_deg;
    uint8_t   fix_type;
    uint8_t   satellites;
    float     hdop;
    uint8_t   _gps_pad[2];           // explicit zero padding

    // YOLO detection summary for this frame
    uint8_t   detection_count;
    float     best_yolo_conf;         // -1.0 if no detections

    // Depth map integrity fingerprint (truncated SHA-256 of MiDaS output floats)
    // Zero-filled when VIGIA_HAVE_MBEDTLS is not defined.
    uint8_t   depth_hash_trunc[8];
};
// Actual packed size: 101 bytes. static_assert keeps layout honest.
static_assert(sizeof(FrameMetadata) == 101,
    "FrameMetadata layout changed — update MsgPack schema and server deserializer");

// Ring header + frame array — the entire shm region.
struct FrameMetadataRing {
    alignas(64) std::atomic<uint32_t> seq{0};  // seqlock counter (even=stable, odd=writing)
    uint32_t write_idx{0};                     // absolute monotonic frame counter
    FrameMetadata frames[kMetaRingDepth];

    // Writer call (FusionNode): seqlock write, never blocks.
    void write_metadata(const FrameMetadata & meta) noexcept {
        seq.fetch_add(1, std::memory_order_release);   // odd → writing
        frames[write_idx % kMetaRingDepth] = meta;
        ++write_idx;
        seq.fetch_add(1, std::memory_order_release);   // even → stable
    }

    // Reader call (AntiDeathNode): seqlock retry loop, wait-free bounded by writer rate.
    void snapshot(std::array<FrameMetadata, kMetaRingDepth> & out,
                  uint32_t & out_write_idx) noexcept
    {
        uint32_t s1 = 0, s2 = 0;
        do {
            s1 = seq.load(std::memory_order_acquire);
            if (s1 & 1u) continue;   // writer in progress — spin
            std::memcpy(out.data(), frames, sizeof(frames));
            out_write_idx = write_idx;
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq.load(std::memory_order_relaxed);
        } while (s1 != s2);
    }
};

// RAII wrapper — maps /vigia_meta_ring into process address space.
class ShmMetaRing {
public:
    // creator = true → shm_open O_CREAT (writer side — FusionNode)
    // creator = false → open existing (reader side — AntiDeathNode)
    explicit ShmMetaRing(bool creator = false) {
        const int flags = creator
            ? (O_CREAT | O_RDWR)
            : O_RDWR;
        fd_ = ::shm_open("/vigia_meta_ring", flags, 0600);
        if (fd_ < 0) throw std::runtime_error("ShmMetaRing: shm_open failed");
        if (creator) {
            if (::ftruncate(fd_, sizeof(FrameMetadataRing)) != 0)
                throw std::runtime_error("ShmMetaRing: ftruncate failed");
        }
        ring_ = static_cast<FrameMetadataRing*>(
            ::mmap(nullptr, sizeof(FrameMetadataRing),
                   PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (ring_ == MAP_FAILED) throw std::runtime_error("ShmMetaRing: mmap failed");
        if (creator) {
            // Placement-new to initialize atomics — required for shared memory.
            new (ring_) FrameMetadataRing{};
        }
    }

    ~ShmMetaRing() {
        if (ring_ && ring_ != MAP_FAILED) ::munmap(ring_, sizeof(FrameMetadataRing));
        if (fd_ >= 0) ::close(fd_);
    }

    FrameMetadataRing * ring() const { return ring_; }

    ShmMetaRing(const ShmMetaRing &) = delete;
    ShmMetaRing & operator=(const ShmMetaRing &) = delete;

private:
    int fd_{-1};
    FrameMetadataRing * ring_{nullptr};
};
