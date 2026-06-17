#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

// Seqlock-protected ring buffer on /dev/shm.
// Writer: CameraNode  (SCHED_FIFO 80)
// Reader: AntiDeathNode (SCHED_FIFO 99) — wait-free snapshot, no mutex.
//
// Seqlock invariant:
//   seq even  → buffer is stable (reader may copy)
//   seq odd   → writer is mid-write (reader must retry)
//   seq change between two reads → concurrent write (reader must retry)
struct ShmFrameSlot {
    std::atomic<uint32_t> seq{0};   // seqlock counter
    uint64_t              timestamp_us{0};
    uint32_t              frame_id{0};
    uint32_t              width{0};
    uint32_t              height{0};
    uint32_t              channels{3};
    // Pixel data follows in-place — see ShmRingBuffer::pixel_data_of()
};

class ShmRingBuffer {
public:
    explicit ShmRingBuffer(size_t ring_depth, uint32_t frame_w, uint32_t frame_h)
        : ring_depth_(ring_depth), frame_w_(frame_w), frame_h_(frame_h),
          slot_bytes_(sizeof(ShmFrameSlot) + frame_w * frame_h * 3),
          total_bytes_(ring_depth * slot_bytes_)
    {
        shm_fd_ = shm_open("/vigia_frame_ring", O_CREAT | O_RDWR, 0600);
        if (shm_fd_ < 0) throw std::runtime_error("shm_open failed");
        if (ftruncate(shm_fd_, static_cast<off_t>(total_bytes_)) != 0)
            throw std::runtime_error("ftruncate failed");
        base_ = static_cast<uint8_t*>(
            mmap(nullptr, total_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
        if (base_ == MAP_FAILED) throw std::runtime_error("mmap failed");
    }

    ~ShmRingBuffer() {
        munmap(base_, total_bytes_);
        close(shm_fd_);
    }

    // Called by CameraNode — never blocks, never allocates.
    void write_frame(uint32_t frame_id, uint64_t ts_us,
                     const uint8_t* bgr_data, size_t data_bytes)
    {
        size_t slot_idx = frame_id % ring_depth_;
        auto* slot = slot_at(slot_idx);

        // Seqlock write-begin: set seq to odd.
        slot->seq.fetch_add(1, std::memory_order_release);

        slot->frame_id      = frame_id;
        slot->timestamp_us  = ts_us;
        slot->width         = frame_w_;
        slot->height        = frame_h_;
        std::memcpy(pixel_data_of(slot), bgr_data,
                    std::min(data_bytes, frame_w_ * frame_h_ * 3UL));

        // Seqlock write-end: set seq back to even.
        slot->seq.fetch_add(1, std::memory_order_release);
    }

    // Called by AntiDeathNode — wait-free read of a single slot by index.
    // Returns true if snapshot is consistent; false if writer was concurrent (retry).
    bool snapshot(size_t slot_idx, ShmFrameSlot& out_meta,
                  uint8_t* out_pixels, size_t pixels_cap) const
    {
        auto* slot = slot_at(slot_idx);
        uint32_t seq1 = slot->seq.load(std::memory_order_acquire);
        if (seq1 & 1u) return false;  // writer in progress

        out_meta = *slot;  // struct copy (no pixel data yet)
        std::memcpy(out_pixels, pixel_data_of(slot),
                    std::min(pixels_cap, static_cast<size_t>(frame_w_ * frame_h_ * 3)));

        uint32_t seq2 = slot->seq.load(std::memory_order_acquire);
        return (seq1 == seq2);  // false → writer touched slot during our copy → retry
    }

    size_t ring_depth() const { return ring_depth_; }

private:
    ShmFrameSlot* slot_at(size_t idx) const {
        return reinterpret_cast<ShmFrameSlot*>(base_ + idx * slot_bytes_);
    }
    uint8_t* pixel_data_of(ShmFrameSlot* slot) const {
        return reinterpret_cast<uint8_t*>(slot) + sizeof(ShmFrameSlot);
    }
    const uint8_t* pixel_data_of(const ShmFrameSlot* slot) const {
        return reinterpret_cast<const uint8_t*>(slot) + sizeof(ShmFrameSlot);
    }

    int      shm_fd_{-1};
    uint8_t* base_{nullptr};
    size_t   ring_depth_;
    uint32_t frame_w_;
    uint32_t frame_h_;
    size_t   slot_bytes_;
    size_t   total_bytes_;
};
