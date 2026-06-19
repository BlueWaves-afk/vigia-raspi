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
// TWO seqlock levels:
//   Per-slot seq  — protects individual slot writes (CameraNode path, unchanged).
//   Global seq    — protects bulk ring snapshot (AntiDeathNode only).
//
// Bulk snapshot invariant (spec §7):
//   The global seqlock allows AntiDeathNode to capture all N slots as one
//   consistent snapshot, not N independent per-slot reads. Without it the writer
//   could advance the ring head between two per-slot reads, giving the reader a
//   mix of frames from two different writer passes.
//
// Usage:
//   Writer calls write_frame() — acquires global seq, writes per-slot seq, writes
//   pixels, releases per-slot seq, releases global seq.
//   Reader calls snapshot_all() — spins until global seq is even, bulk-copies,
//   verifies global seq unchanged. Returns consistent frame vector or retries.
struct ShmFrameSlot {
    std::atomic<uint32_t> seq{0};   // per-slot seqlock counter
    uint64_t              timestamp_us{0};
    uint32_t              frame_id{0};
    uint32_t              width{0};
    uint32_t              height{0};
    uint32_t              channels{3};
    // Pixel data follows in-place — see ShmRingBuffer::pixel_data_of()
};

// Header at the front of the shared memory region, before the slot array.
struct ShmRingHeader {
    std::atomic<uint32_t> global_seq{0};   // global seqlock — even = stable, odd = writing
    uint32_t              ring_depth{0};
    uint32_t              frame_w{0};
    uint32_t              frame_h{0};
    uint32_t              write_head{0};   // index of the most recently written slot
    uint8_t               _pad[44];        // cache-line pad to 64 bytes
};
static_assert(sizeof(ShmRingHeader) == 64, "ShmRingHeader must be 64 bytes");

class ShmRingBuffer {
public:
    explicit ShmRingBuffer(size_t ring_depth, uint32_t frame_w, uint32_t frame_h)
        : ring_depth_(ring_depth), frame_w_(frame_w), frame_h_(frame_h),
          slot_bytes_(sizeof(ShmFrameSlot) + frame_w * frame_h * 3),
          total_bytes_(sizeof(ShmRingHeader) + ring_depth * slot_bytes_)
    {
        shm_fd_ = shm_open("/vigia_frame_ring", O_CREAT | O_RDWR, 0600);
        if (shm_fd_ < 0) throw std::runtime_error("shm_open failed");
        if (ftruncate(shm_fd_, static_cast<off_t>(total_bytes_)) != 0)
            throw std::runtime_error("ftruncate failed");
        base_ = static_cast<uint8_t*>(
            mmap(nullptr, total_bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
        if (base_ == MAP_FAILED) throw std::runtime_error("mmap failed");

        // Initialise header on first creation (idempotent — zeros are valid initial state)
        auto* hdr = header();
        hdr->ring_depth = static_cast<uint32_t>(ring_depth);
        hdr->frame_w    = frame_w;
        hdr->frame_h    = frame_h;
    }

    ~ShmRingBuffer() {
        munmap(base_, total_bytes_);
        close(shm_fd_);
    }

    // Called by CameraNode — never blocks, never allocates.
    // Acquires the global seqlock around both the per-slot write AND the header
    // write_head update so snapshot_all() gets a coherent ring view.
    void write_frame(uint32_t frame_id, uint64_t ts_us,
                     const uint8_t* bgr_data, size_t data_bytes)
    {
        auto* hdr      = header();
        size_t slot_idx = frame_id % ring_depth_;
        auto*  slot    = slot_at(slot_idx);

        // Global seqlock: begin write (odd)
        hdr->global_seq.fetch_add(1, std::memory_order_release);

        // Per-slot seqlock: begin write (odd)
        slot->seq.fetch_add(1, std::memory_order_release);

        slot->frame_id     = frame_id;
        slot->timestamp_us = ts_us;
        slot->width        = frame_w_;
        slot->height       = frame_h_;
        std::memcpy(pixel_data_of(slot), bgr_data,
                    std::min(data_bytes, frame_w_ * frame_h_ * 3UL));

        // Per-slot seqlock: end write (even)
        slot->seq.fetch_add(1, std::memory_order_release);

        hdr->write_head = static_cast<uint32_t>(slot_idx);

        // Global seqlock: end write (even)
        hdr->global_seq.fetch_add(1, std::memory_order_release);
    }

    // Called by AntiDeathNode — wait-free bulk snapshot of the entire ring.
    // Spins retrying until it captures a consistent view (global seq unchanged).
    // out_slots must be pre-allocated (ring_depth entries); out_pixels is a flat
    // buffer of ring_depth * frame_w * frame_h * 3 bytes.
    // Returns the write_head index (most recently written slot) on success.
    uint32_t snapshot_all(std::vector<ShmFrameSlot>& out_slots,
                          uint8_t* out_pixels, size_t pixels_cap) const
    {
        auto* hdr = header();
        out_slots.resize(ring_depth_);

        while (true) {
            uint32_t g1 = hdr->global_seq.load(std::memory_order_acquire);
            if (g1 & 1u) continue;   // writer in progress — spin

            // Bulk copy all slots
            uint32_t write_head = hdr->write_head;
            for (size_t i = 0; i < ring_depth_; ++i) {
                auto* slot = slot_at(i);
                out_slots[i].timestamp_us = slot->timestamp_us;
                out_slots[i].frame_id     = slot->frame_id;
                out_slots[i].width        = slot->width;
                out_slots[i].height       = slot->height;
                out_slots[i].channels     = slot->channels;

                const size_t poff = i * frame_w_ * frame_h_ * 3;
                if (poff + frame_w_ * frame_h_ * 3 <= pixels_cap) {
                    std::memcpy(out_pixels + poff, pixel_data_of(slot),
                                frame_w_ * frame_h_ * 3);
                }
            }

            uint32_t g2 = hdr->global_seq.load(std::memory_order_acquire);
            if (g1 == g2) return write_head;   // consistent — done
            // Writer touched the ring during our copy — retry
        }
    }

    // Single-slot read (per-slot seqlock only — used by non-AntiDeath readers).
    bool snapshot(size_t slot_idx, ShmFrameSlot& out_meta,
                  uint8_t* out_pixels, size_t pixels_cap) const
    {
        auto* slot = slot_at(slot_idx);
        uint32_t seq1 = slot->seq.load(std::memory_order_acquire);
        if (seq1 & 1u) return false;

        out_meta.timestamp_us = slot->timestamp_us;
        out_meta.frame_id     = slot->frame_id;
        out_meta.width        = slot->width;
        out_meta.height       = slot->height;
        out_meta.channels     = slot->channels;
        std::memcpy(out_pixels, pixel_data_of(slot),
                    std::min(pixels_cap, static_cast<size_t>(frame_w_ * frame_h_ * 3)));

        uint32_t seq2 = slot->seq.load(std::memory_order_acquire);
        return (seq1 == seq2);
    }

    size_t   ring_depth() const { return ring_depth_; }
    uint32_t write_head() const { return header()->write_head; }

private:
    ShmRingHeader* header() const {
        return reinterpret_cast<ShmRingHeader*>(base_);
    }
    ShmFrameSlot* slot_at(size_t idx) const {
        return reinterpret_cast<ShmFrameSlot*>(
            base_ + sizeof(ShmRingHeader) + idx * slot_bytes_);
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
