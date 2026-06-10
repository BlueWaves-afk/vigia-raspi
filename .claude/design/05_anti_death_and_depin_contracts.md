# VIGIA ADAS DePIN Edge Node
## Anti-Death Storage & DePIN Attestation Contracts
**Document:** `05_anti_death_and_depin_contracts.md`  
**Depends on:** `01`–`04` (all APPROVED)  
**Status:** AWAITING APPROVAL — No implementation until sign-off  
**Scope:** Phase 5 (Anti-Death Storage & MQTT) + Phase 6 (DePIN Security & Attestation)

---

## 0. Non-Negotiable Emergency Sequence Invariants

| Invariant | Enforcement |
|---|---|
| **`execute_emergency_sequence()` never yields to ROS 2** | It runs synchronously on the `vigia_antideath` thread (SCHED_FIFO 99). `rclcpp::spin_some()`, `executor.spin()`, and co-routines are forbidden inside the sequence. |
| **Monotonic clock for all budget checks** | `std::chrono::steady_clock` only. `CLOCK_REALTIME` is susceptible to NTP jumps; steady_clock is immune. |
| **Single MsgPack allocation permitted** | `msgpack::sbuffer` construction is the one sanctioned heap allocation. All other data (snapshot structs, Paho client, TLS context, GPIO handle) is pre-allocated at node startup. |
| **Paho `async_client` must be pre-connected at node startup** | Connecting to the MQTT broker for the first time during the emergency sequence wastes 2–5 seconds of the power window on DNS and TCP setup. The client connects at startup, keeps the session alive with PINGREQ/PINGRESP, and calls only `publish()` during the emergency. |
| **`execute_emergency_sequence()` is not reentrant** | Protected by `std::atomic_flag emergency_in_progress_`. A second GPIO assertion during an ongoing sequence is silently ignored — we are already executing. |
| **Raw frame pixels are NOT transmitted** | The 829 MB `/dev/shm` frame buffer cannot be transmitted over LTE in 15 seconds. The MQTT payload carries the compact **frame metadata sidecar** (§3), the latest `SpatialLatent` (S_t), and the STM32-signed `SignedEt` (E_t). Frame pixels serve as local forensic evidence until the power cut eliminates them. |

---

## 1. Startup Initialization (Before Emergency Arises)

All expensive, potentially blocking operations happen at node construction — never inside `execute_emergency_sequence()`.

```cpp
// anti_death_node.cpp — AntiDeathNode constructor

void AntiDeathNode::startup_init() {
    // ── 1. Pre-allocate latent cache buffer ───────────────────────────────
    // We maintain a local copy of the latest S_t and S_t−1 for the snapshot.
    // Two buffers allow the subscriber callback to write the new S_t while
    // execute_emergency_sequence() reads the previous one without locking.
    latest_latent_a_.latent_vector.reserve(VisionNode::kMaxLatentElems);
    latest_latent_b_.latent_vector.reserve(VisionNode::kMaxLatentElems);
    active_latent_buf_ = &latest_latent_a_;

    // ── 2. Read device_id from /etc/vigia/device_id ───────────────────────
    std::ifstream id_file("/etc/vigia/device_id");
    id_file >> device_id_;      // UUID string, e.g. "550e8400-e29b-41d4-a716-446655440000"

    // ── 3. Initialize SIM7600 and establish LTE data bearer ───────────────
    sim7600_init();             // AT commands → ECM mode → usb0 interface up (§5.1)

    // ── 4. Pre-connect Paho MQTT client ───────────────────────────────────
    mqtt_client_init();         // Creates async_client, calls connect(), blocks until CONNACK (§6)

    // ── 5. Map /dev/shm ring buffer ───────────────────────────────────────
    shm_ring_ = shm_map_ring_buffer();   // mmap of ShmRingBuffer (829 MB)
    if (!shm_ring_) {
        RCLCPP_FATAL(get_logger(), "Failed to map /dev/shm ring buffer. Aborting.");
        throw std::runtime_error("shm_map_ring_buffer failed");
    }

    // ── 6. Map /dev/shm metadata sidecar ─────────────────────────────────
    meta_ring_ = shm_map_metadata_ring();   // mmap of FrameMetadataRing (§3)

    // ── 7. Initialize libgpiod for UPS GPIO ───────────────────────────────
    gpio_chip_  = gpiod_chip_open(declare_parameter<std::string>("ups_gpio_chip").c_str());
    gpio_line_  = gpiod_chip_get_line(gpio_chip_, declare_parameter<int>("ups_gpio_line"));
    gpiod_line_request_falling_edge_events(gpio_line_, "vigia_antideath");

    // ── 8. Start GPIO monitoring timer (1 ms poll) ───────────────────────
    gpio_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&AntiDeathNode::gpio_poll_callback, this));

    RCLCPP_INFO(get_logger(),
        "AntiDeathNode armed. MQTT connected to %s. GPIO line %d monitoring.",
        mqtt_broker_host_.c_str(), ups_gpio_line_);
}
```

---

## 2. ROS 2 Subscriber Callbacks — Cache-Only, No Logic

All three subscriber callbacks do one thing: update the cached "latest" state. No business logic runs here. The double-buffer pattern for `SpatialLatent` avoids a lock between the callback (SCHED_FIFO 99 context when the executor fires) and `execute_emergency_sequence()`.

```cpp
// Spatial latent — double-buffer swap (amortized zero-alloc after first frame)
void AntiDeathNode::on_spatial_latent(
    std::unique_ptr<vigia_msgs::msg::SpatialLatent> msg)
{
    // Write into the INACTIVE buffer (not the one execute_emergency_sequence reads)
    auto* write_buf = (active_latent_buf_ == &latest_latent_a_)
                      ? &latest_latent_b_ : &latest_latent_a_;

    write_buf->header            = msg->header;
    write_buf->frame_id          = msg->frame_id;
    write_buf->source_layer_name = msg->source_layer_name;
    write_buf->latent_vector.resize(msg->latent_vector.size());
    std::memcpy(write_buf->latent_vector.data(),
                msg->latent_vector.data(),
                msg->latent_vector.size() * sizeof(float));

    // Atomic pointer swap — emergency sequence reads active_latent_buf_
    active_latent_buf_.store(write_buf, std::memory_order_release);
    // msg destroyed here — its unique_ptr releases the VisionNode allocation
}

// SignedEt — direct swap; struct is small (~200 B), copy is negligible
void AntiDeathNode::on_signed_et(
    std::shared_ptr<const vigia_msgs::msg::SignedEt> msg)
{
    latest_signed_et_ = *msg;  // value copy into pre-allocated struct member
    signed_et_valid_  = true;
}

// HazardEvent — overwrite with latest
void AntiDeathNode::on_hazard_event(
    std::unique_ptr<vigia_msgs::msg::HazardEvent> msg)
{
    latest_hazard_event_ = std::move(msg);  // replaces previous unique_ptr
}
```

---

## 3. Frame Metadata Sidecar Ring

The raw frame pixels in `ShmRingBuffer` (829 MB) are NOT transmitted. A parallel compact ring buffer — `FrameMetadataRing` — stores per-frame sensor context. This is what gets serialized into the MQTT payload.

```cpp
// vigia_edge_node/include/shm_ring_buffer.hpp (addition)

// Per-frame metadata — written by FusionNode alongside CameraNode frame writes.
// Size: 124 bytes per frame × 300 frames = 37,200 bytes ≈ 36 KB total.
struct __attribute__((packed)) FrameMetadata {
    uint64_t  timestamp_us;           // CameraNode capture time
    uint32_t  frame_id;               // Monotonic frame counter
    float     rri_score;              // Road Risk Index [0,1] (-1.0 = not computed yet)
    float     iss_score;              // Impact Severity Score (-1.0 = not computed)
    // IMU fields (8 bytes float × 7 = 56 bytes + 1 byte cal = 57 bytes → padded to 60)
    float     q_w, q_x, q_y, q_z;
    float     lin_accel_x, lin_accel_y, lin_accel_z;
    uint8_t   imu_cal_status;
    uint8_t   _imu_pad[3];
    // GPS fields (36 bytes + alignment)
    double    latitude, longitude;    // 16 bytes
    float     altitude_m, speed_ms, course_deg;  // 12 bytes
    uint8_t   fix_type, satellites;
    float     hdop;                   // 4 bytes
    uint8_t   _gps_pad[2];
    // Detection summary
    uint8_t   detection_count;        // Number of YOLO detections this frame
    float     best_yolo_conf;         // Highest confidence detection (-1.0 if none)
    // Depth map integrity hash (SHA-256 truncated to 8 bytes for compactness)
    uint8_t   depth_hash_trunc[8];
};
static_assert(sizeof(FrameMetadata) == 124, "FrameMetadata layout changed");

struct FrameMetadataRing {
    alignas(64) std::atomic<uint32_t> seq{0};  // seqlock (mirrors ShmRingBuffer.seq)
    uint32_t     write_idx{0};
    FrameMetadata frames[ShmRingBuffer::kRingDepth];  // kRingDepth = 300

    // Writer: FusionNode calls this on every processed frame
    void write_metadata(const FrameMetadata& meta) {
        seq.fetch_add(1, std::memory_order_release);
        frames[write_idx % ShmRingBuffer::kRingDepth] = meta;
        write_idx++;
        seq.fetch_add(1, std::memory_order_release);
    }

    // Snapshot: AntiDeathNode — same seqlock protocol as ShmRingBuffer
    void snapshot(std::array<FrameMetadata, ShmRingBuffer::kRingDepth>& out) {
        uint32_t s1, s2;
        do {
            s1 = seq.load(std::memory_order_acquire);
            if (s1 & 1u) continue;
            std::memcpy(out.data(), frames, sizeof(frames));
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = seq.load(std::memory_order_relaxed);
        } while (s1 != s2);
    }
};
// Size: ~37.2 KB — fits comfortably in POSIX shared memory alongside ShmRingBuffer
// Allocated at: /dev/shm/vigia_meta_ring.buf (mmap'd by CameraNode + FusionNode + AntiDeathNode)
```

`FrameMetadataRing` is mmap-shared between:
- **FusionNode** (writer via `write_metadata()` after each RRI computation)
- **CameraNode** (writes `timestamp_us`, `frame_id`, `imu_cal_status` on frame capture)
- **AntiDeathNode** (snapshot reader during emergency)

---

## 4. Emergency State Machine

### 4.1 State Enum and Time Budget Table

```cpp
// anti_death_node.hpp

enum class EmergencyState : uint8_t {
    IDLE               = 0,
    CAPTURING_SNAPSHOT = 1,   // seqlock snapshot of metadata + latent + signed_et
    SERIALIZING        = 2,   // msgpack encode into heap buffer
    MQTT_CONNECTING    = 3,   // ensure Paho client is connected (usually pre-connected)
    MQTT_TRANSMITTING  = 4,   // publish() + wait for PUBACK
    SAFE_SHUTDOWN      = 5,   // rclcpp::shutdown + sync
    ABORTED            = 6,   // budget exceeded in any state → skip to SAFE_SHUTDOWN
};

// Absolute time budgets (from T=0, the UPS GPIO assertion instant):
//   State entry MUST begin by this deadline; exit MUST complete by next deadline.
struct StateBudget {
    static constexpr double kCaptureDeadlineS   = 1.0;  // CAPTURING_SNAPSHOT must finish by T+1.0s
    static constexpr double kSerializeDeadlineS = 3.0;  // SERIALIZING must finish by T+3.0s
    static constexpr double kConnectDeadlineS   = 8.0;  // MQTT_CONNECTING must finish by T+8.0s
    static constexpr double kTransmitDeadlineS  = 13.0; // MQTT_TRANSMITTING must finish by T+13.0s
    static constexpr double kPowerWindowS       = 15.0; // Physical power loss deadline
    // Safety margin: 2.0s between our target and actual power loss
};
```

### 4.2 GPIO Poll Callback — Entry Trigger

```cpp
// anti_death_node.cpp
// Called every 1 ms by the StaticSingleThreadedExecutor timer on Core 3.

void AntiDeathNode::gpio_poll_callback() {
    // Non-blocking edge check: returns immediately if no event
    if (!gpiod_line_event_wait(gpio_line_, /*timeout=*/&kZeroTimeout)) return;

    gpiod_line_event event;
    if (gpiod_line_event_read(gpio_line_, &event) != 0)         return;
    if (event.event_type != GPIOD_LINE_EVENT_FALLING_EDGE)      return;

    // Atomic guard: reject concurrent re-entry
    bool expected = false;
    if (!emergency_in_progress_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        RCLCPP_WARN(get_logger(), "UPS re-assertion during active sequence — ignored.");
        return;
    }

    // Record T=0 (absolute epoch for all budget checks)
    t_zero_ = std::chrono::steady_clock::now();
    RCLCPP_FATAL(get_logger(),
        "UPS POWER_FAIL asserted at T=0. Emergency sequence initiated.");

    // Transfer control to the state machine.
    // This call blocks until SAFE_SHUTDOWN; it never returns to the executor.
    execute_emergency_sequence();
}
```

### 4.3 `execute_emergency_sequence()` — Top-Level Dispatcher

```cpp
void AntiDeathNode::execute_emergency_sequence() {
    // All state transitions are sequential — no async, no yields.
    // This function executes entirely on the vigia_antideath thread (SCHED_FIFO 99).

    EmergencyState state = EmergencyState::CAPTURING_SNAPSHOT;
    msgpack::sbuffer payload_buf;  // THE ONE PERMITTED HEAP ALLOCATION

    SnapshotData snapshot{};       // stack-allocated POD struct (see §4.4)

    while (state != EmergencyState::SAFE_SHUTDOWN &&
           state != EmergencyState::ABORTED)
    {
        const double elapsed = elapsed_seconds_since_t0();

        switch (state) {

        case EmergencyState::CAPTURING_SNAPSHOT:
            if (elapsed > StateBudget::kCaptureDeadlineS) {
                RCLCPP_ERROR(get_logger(), "BUDGET EXCEEDED: CAPTURING_SNAPSHOT at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_capture_snapshot(snapshot) ? EmergencyState::SERIALIZING
                                                   : EmergencyState::ABORTED;
            break;

        case EmergencyState::SERIALIZING:
            if (elapsed > StateBudget::kSerializeDeadlineS) {
                RCLCPP_ERROR(get_logger(), "BUDGET EXCEEDED: SERIALIZING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_serialize(snapshot, payload_buf) ? EmergencyState::MQTT_CONNECTING
                                                        : EmergencyState::ABORTED;
            break;

        case EmergencyState::MQTT_CONNECTING:
            if (elapsed > StateBudget::kConnectDeadlineS) {
                RCLCPP_ERROR(get_logger(), "BUDGET EXCEEDED: MQTT_CONNECTING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            state = do_mqtt_connect() ? EmergencyState::MQTT_TRANSMITTING
                                      : EmergencyState::ABORTED;
            break;

        case EmergencyState::MQTT_TRANSMITTING:
            if (elapsed > StateBudget::kTransmitDeadlineS) {
                RCLCPP_ERROR(get_logger(), "BUDGET EXCEEDED: MQTT_TRANSMITTING at T+%.2fs", elapsed);
                state = EmergencyState::ABORTED; break;
            }
            // Transmission may or may not succeed — proceed to shutdown regardless
            do_mqtt_transmit(payload_buf);
            state = EmergencyState::SAFE_SHUTDOWN;
            break;

        default:
            state = EmergencyState::SAFE_SHUTDOWN;
        }
    }

    do_safe_shutdown();
}

inline double AntiDeathNode::elapsed_seconds_since_t0() const {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_zero_).count();
}
```

### 4.4 State: `CAPTURING_SNAPSHOT` (Budget: T+0 → T+1.0s)

```cpp
// Stack-allocated snapshot container — all fields are POD or fixed-size arrays.
// sizeof(SnapshotData) ≈ 37 KB (dominated by FrameMetadata array).
struct SnapshotData {
    // Seqlock-consistent copy of the metadata sidecar
    std::array<FrameMetadata, ShmRingBuffer::kRingDepth> frame_metadata{};
    uint32_t  metadata_write_idx{0};    // snapshot of write_idx at capture time

    // Latest spatial latent (pointer into pre-allocated double buffer — NOT a copy)
    // Points to active_latent_buf_ value frozen at snapshot time.
    const vigia_msgs::msg::SpatialLatent* spatial_latent_snapshot{nullptr};

    // Latest SignedEt (value copy — small struct, ~200 B)
    vigia_msgs::msg::SignedEt signed_et_snapshot{};
    bool signed_et_valid{false};

    // Latest HazardEvent (pointer — unique_ptr is owned by AntiDeathNode)
    const vigia_msgs::msg::HazardEvent* hazard_event_snapshot{nullptr};

    // Capture metadata
    uint64_t capture_timestamp_us{0};
    char     device_id[64]{};
};

bool AntiDeathNode::do_capture_snapshot(SnapshotData& snap) {
    const auto t_capture_start = std::chrono::steady_clock::now();

    // ── 1. Seqlock snapshot of FrameMetadataRing (~36 KB memcpy) ─────────
    meta_ring_->snapshot(snap.frame_metadata);
    snap.metadata_write_idx = meta_ring_->write_idx;
    // Expected duration: ~15 µs (36 KB × 1 copy at LPDDR4X bandwidth)

    // ── 2. Freeze spatial latent pointer ─────────────────────────────────
    // atomic_load of the active double-buffer pointer — no copy of the data.
    // The latent data is read in-place during SERIALIZING; we hold the pointer.
    snap.spatial_latent_snapshot =
        active_latent_buf_.load(std::memory_order_acquire);
    // This is safe: subscriber callbacks write to the INACTIVE buffer and then
    // swap the pointer. Once we freeze the pointer here, SERIALIZING reads from
    // the buffer that the callback is NOT currently writing to.

    // ── 3. Copy SignedEt (value copy, ~200 bytes) ─────────────────────────
    snap.signed_et_snapshot = latest_signed_et_;
    snap.signed_et_valid     = signed_et_valid_;

    // ── 4. Freeze HazardEvent pointer ────────────────────────────────────
    snap.hazard_event_snapshot = latest_hazard_event_.get();

    // ── 5. Record capture metadata ────────────────────────────────────────
    snap.capture_timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            t_zero_.time_since_epoch()).count();
    std::strncpy(snap.device_id, device_id_.c_str(), sizeof(snap.device_id) - 1);

    const double capture_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_capture_start).count();

    RCLCPP_INFO(get_logger(),
        "CAPTURING_SNAPSHOT complete in %.2f ms. "
        "Frames: %u, Latent: %zu elements, SignedEt valid: %s",
        capture_ms,
        snap.metadata_write_idx,
        snap.spatial_latent_snapshot
            ? snap.spatial_latent_snapshot->latent_vector.size() : 0UL,
        snap.signed_et_valid ? "YES" : "NO");

    return true;  // snapshot always succeeds (never involves I/O)
}
```

### 4.5 State: `SERIALIZING` (Budget: T+1.0s → T+3.0s)

MsgPack schema and serialization. See §5 for complete field-level schema definition.

```cpp
bool AntiDeathNode::do_serialize(
    const SnapshotData& snap,
    msgpack::sbuffer& buf)
{
    // Pre-estimate payload size to avoid sbuffer internal realloc mid-pack:
    //   Frame metadata: 300 × 124 B ≈ 37 KB
    //   Spatial latent: 102400 × 4 B ≈ 400 KB (packed as msgpack bin)
    //   SignedEt:       ~200 B
    //   HazardEvent:    ~2 KB
    //   Overhead:       ~4 KB
    //   Total estimate: ~443 KB
    buf.reserve(500 * 1024);  // 500 KB — single upfront allocation, no realloc

    msgpack::packer<msgpack::sbuffer> pk(buf);

    // Root: map with 7 top-level keys
    pk.pack_map(7);

    // ── "version" ──────────────────────────────────────────────────────────
    pk.pack("version");
    pk.pack(uint8_t{1});

    // ── "device_id" ───────────────────────────────────────────────────────
    pk.pack("device_id");
    pk.pack(std::string(snap.device_id));

    // ── "capture_timestamp_us" ────────────────────────────────────────────
    pk.pack("capture_timestamp_us");
    pk.pack(snap.capture_timestamp_us);

    // ── "frame_metadata" (array of FrameMetadata structs) ─────────────────
    pk.pack("frame_metadata");
    pack_frame_metadata_array(pk, snap);

    // ── "spatial_latent" (raw binary — NOT a msgpack float array) ────────
    // Rationale: a msgpack float32 array encodes each element as 5 bytes
    // (1-byte type tag + 4 bytes IEEE754). Packing as msgpack bin (raw bytes)
    // encodes as 4 bytes per element — 20% smaller for 102K elements.
    pk.pack("spatial_latent");
    if (snap.spatial_latent_snapshot &&
        !snap.spatial_latent_snapshot->latent_vector.empty())
    {
        const auto& lv = snap.spatial_latent_snapshot->latent_vector;
        const size_t byte_len = lv.size() * sizeof(float);
        pk.pack_bin(static_cast<uint32_t>(byte_len));
        pk.pack_bin_body(
            reinterpret_cast<const char*>(lv.data()),
            byte_len);
        // Endianness note: floats are packed as native little-endian bytes.
        // Server-side deserialization must interpret as LE float32.
        // Documented in §8.1.
    } else {
        pk.pack_nil();
    }

    // ── "signed_et" ───────────────────────────────────────────────────────
    pk.pack("signed_et");
    pack_signed_et(pk, snap.signed_et_snapshot, snap.signed_et_valid);

    // ── "hazard_event" ────────────────────────────────────────────────────
    pk.pack("hazard_event");
    if (snap.hazard_event_snapshot) {
        pack_hazard_event(pk, *snap.hazard_event_snapshot);
    } else {
        pk.pack_nil();
    }

    RCLCPP_INFO(get_logger(),
        "SERIALIZING complete. Payload: %.1f KB at T+%.2fs",
        buf.size() / 1024.0, elapsed_seconds_since_t0());

    return true;
}
```

### 4.6 State: `MQTT_CONNECTING` (Budget: T+3.0s → T+8.0s)

Since the client is pre-connected at startup, this state is usually instantaneous — it verifies connectivity and attempts reconnection only if the broker dropped the persistent session.

```cpp
bool AntiDeathNode::do_mqtt_connect() {
    // Fast path: client is pre-connected from startup_init().
    if (mqtt_client_->is_connected()) {
        RCLCPP_INFO(get_logger(),
            "MQTT_CONNECTING: broker already connected. T+%.2fs", elapsed_seconds_since_t0());
        return true;
    }

    // Slow path: connection was dropped (LTE handoff, broker restart).
    // Attempt reconnect with deadline-aware retry loop.
    RCLCPP_WARN(get_logger(), "MQTT client disconnected. Reconnecting...");

    constexpr int kMaxRetries = 3;
    for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
        if (elapsed_seconds_since_t0() > StateBudget::kConnectDeadlineS) {
            RCLCPP_ERROR(get_logger(), "MQTT_CONNECTING: deadline exceeded mid-retry.");
            return false;
        }

        try {
            auto tok = mqtt_client_->connect(conn_opts_);
            // wait_for with deadline-relative timeout
            const double budget_remaining =
                StateBudget::kConnectDeadlineS - elapsed_seconds_since_t0();
            const bool connected =
                tok->wait_for(std::chrono::duration<double>(budget_remaining / kMaxRetries));

            if (connected && mqtt_client_->is_connected()) {
                RCLCPP_INFO(get_logger(),
                    "MQTT reconnected on attempt %d. T+%.2fs",
                    attempt, elapsed_seconds_since_t0());
                return true;
            }
        } catch (const mqtt::exception& ex) {
            RCLCPP_WARN(get_logger(), "MQTT connect attempt %d failed: %s", attempt, ex.what());
        }
    }

    RCLCPP_ERROR(get_logger(), "MQTT_CONNECTING: all %d retries exhausted.", kMaxRetries);
    return false;
}
```

### 4.7 State: `MQTT_TRANSMITTING` (Budget: T+8.0s → T+13.0s)

```cpp
void AntiDeathNode::do_mqtt_transmit(const msgpack::sbuffer& buf) {
    const std::string topic =
        mqtt_topic_prefix_ + "/" + device_id_ + "/hazard";

    RCLCPP_INFO(get_logger(),
        "MQTT_TRANSMITTING: publishing %.1f KB to '%s'. T+%.2fs",
        buf.size() / 1024.0, topic.c_str(), elapsed_seconds_since_t0());

    try {
        // mqtt::make_message: copies buf data into a Paho-managed buffer.
        // This is the SECOND allocation in the emergency sequence (unavoidable —
        // Paho owns the message lifecycle through async I/O completion).
        auto msg = mqtt::make_message(
            topic,
            buf.data(),        // const void*
            buf.size(),        // payload byte count
            /*qos=*/1,         // QoS 1: at-least-once (PUBACK required before return)
            /*retained=*/false
        );
        msg->set_payload_ref(buf.data(), buf.size());
        // set_payload_ref: zero-copy reference — Paho reads directly from buf.
        // buf remains valid until do_safe_shutdown() because it lives in
        // execute_emergency_sequence()'s stack frame.

        auto publish_token = mqtt_client_->publish(msg);

        // Blocking wait with hard deadline
        const double budget_remaining =
            StateBudget::kTransmitDeadlineS - elapsed_seconds_since_t0();
        const bool ack_received =
            publish_token->wait_for(std::chrono::duration<double>(budget_remaining));

        if (ack_received) {
            RCLCPP_INFO(get_logger(),
                "MQTT PUBACK received. Event attested at T+%.2fs.",
                elapsed_seconds_since_t0());
        } else {
            RCLCPP_WARN(get_logger(),
                "MQTT PUBACK timeout. Payload likely in-flight at power cut. "
                "QoS 1 guarantees at-least-once delivery if broker received packet.");
        }
        // Proceed to SAFE_SHUTDOWN regardless of PUBACK status.
        // QoS 1 session state persists on the broker side; reconnect on next boot
        // will retransmit any unacknowledged inflight message.

    } catch (const mqtt::exception& ex) {
        RCLCPP_ERROR(get_logger(), "MQTT publish exception: %s", ex.what());
        // Fall through to SAFE_SHUTDOWN — we tried.
    }
}
```

### 4.8 State: `SAFE_SHUTDOWN`

```cpp
void AntiDeathNode::do_safe_shutdown() {
    RCLCPP_FATAL(get_logger(),
        "SAFE_SHUTDOWN initiated at T+%.2fs. Power window: %.1fs remaining.",
        elapsed_seconds_since_t0(),
        StateBudget::kPowerWindowS - elapsed_seconds_since_t0());

    // 1. Flush journald ring buffer to ramoops (kernel crash log).
    //    Survives power cut in SoC SRAM; readable after cold reboot via
    //    /sys/fs/pstore/console-ramoops or `journalctl -k` after recovery.
    ::sync();

    // 2. Attempt graceful Paho disconnect (best-effort, may not complete).
    if (mqtt_client_ && mqtt_client_->is_connected()) {
        try {
            mqtt_client_->disconnect()->wait_for(std::chrono::milliseconds(500));
        } catch (...) { /* power may arrive before ACK */ }
    }

    // 3. Shutdown ROS 2 context.
    rclcpp::shutdown();

    // 4. Block here — power cut will interrupt at any point below this line.
    //    The /dev/shm contents are lost on power cut (expected, by design).
    //    The signed E_t payload is in the broker's QoS 1 session if PUBACK was received.
    RCLCPP_FATAL(get_logger(), "Awaiting power cut. Goodbye.");

    // Spin wait — there is nothing else to do.
    while (true) { __asm__ volatile("yield"); }
}
```

---

## 5. MsgPack Payload Schema

The complete schema for the MQTT payload. All numeric types are little-endian (Pi 5 native). The server MUST respect this when deserializing.

```
root: map(7)
├── "version"              : uint8         = 1
├── "device_id"            : str           = UUID string "550e8400-..."
├── "capture_timestamp_us" : uint64        = steady_clock epoch µs since T=0
│
├── "frame_metadata"       : array(≤300)
│   └── [i]: map(12)
│       ├── "frame_id"         : uint32
│       ├── "timestamp_us"     : uint64
│       ├── "rri_score"        : float32   (-1.0 if not yet fused)
│       ├── "iss_score"        : float32
│       ├── "q_w/x/y/z"       : float32 × 4
│       ├── "accel_xyz"        : float32 × 3  (body-frame, gravity included)
│       ├── "imu_cal"          : uint8
│       ├── "lat/lon"          : float64 × 2
│       ├── "alt_m"            : float32
│       ├── "speed_ms"         : float32
│       ├── "fix_type"         : uint8
│       ├── "detection_count"  : uint8
│       ├── "best_yolo_conf"   : float32
│       └── "depth_hash"       : bin(8)    (truncated SHA-256 of depth map)
│
├── "spatial_latent"       : bin(N×4) | nil
│   └── Raw little-endian float32 bytes. Length = latent_n_elems × 4.
│       Server interprets as float32[] by casting: np.frombuffer(data, dtype='<f4')
│
├── "signed_et"            : map(6) | nil
│   ├── "sequence"         : uint32        (monotonic counter — anti-replay key)
│   ├── "stm32_timestamp_us": uint64
│   ├── "et_hash"          : bin(32)       SHA-256(EtHashInput) — see §8.3
│   ├── "ecdsa_sig"        : bin(64)       secp256r1 raw R∥S (32+32 bytes)
│   ├── "imu_cal_status"   : uint8
│   └── "gps_fix_type"     : uint8
│
└── "hazard_event"         : map(8) | nil
    ├── "rri_score"        : float32
    ├── "iss_score"        : float32
    ├── "yolo_confidence"  : float32
    ├── "lat/lon"          : float64 × 2
    ├── "speed_ms"         : float32
    ├── "detection_count"  : uint8
    └── "event_timestamp_us": uint64
```

**Estimated payload sizes:**

| Field | Size | Notes |
|---|---|---|
| `frame_metadata` (300 frames) | ~38 KB | Compact msgpack map encoding |
| `spatial_latent` (102,400 floats as bin) | ~400 KB | `bin(409600)` header + raw bytes |
| `signed_et` | ~170 bytes | Dominated by 32+64 byte crypto fields |
| `hazard_event` | ~100 bytes | Scalar fields only |
| Headers + overhead | ~2 KB | msgpack framing |
| **Total** | **~440 KB** | **At 2 Mbps UL: ~1.8s transmit time** |

---

## 6. SIM7600 LTE Initialization

### 6.1 SIM7600 in ECM Network Interface Mode

The SIM7600 is configured as a Linux network interface (`usb0`) rather than using AT+CIPSEND raw TCP. This routes all traffic through the standard Linux TCP/IP stack, allowing Paho MQTT C++ to use normal POSIX sockets — no AT command interaction at transmit time.

**One-time provisioning** (run once on device setup, persists in SIM7600 NVM):

```
AT                              → OK  (verify modem responsive)
AT+CPIN?                        → +CPIN: READY  (SIM present and unlocked)
AT+CGDCONT=1,"IP","<operator_apn>"  → configure APN for your SIM carrier
AT+CUSBPIDSWITCH=9011,1,1       → switch USB descriptor to ECM mode
                                   (9011 = VID:PID for ECM; reboots modem)
```

After `CUSBPIDSWITCH`, the SIM7600 reboots and enumerates as a USB ECM device. On Pi 5 (Linux 6.6), the `cdc_ether` driver loads automatically and creates a `usb0` interface.

**Per-boot network setup** (in `vigia-edge.service` `ExecStartPre`):

```bash
# vigia-edge.service ExecStartPre= scripts:

# 1. Wait for usb0 to appear (SIM7600 ECM device enumeration)
timeout 30 bash -c 'until ip link show usb0 > /dev/null 2>&1; do sleep 1; done'

# 2. Bring interface up and acquire DHCP address from SIM7600's built-in DHCP server
ip link set usb0 up
dhclient -1 -v usb0

# 3. Add a specific host route for the MQTT broker via usb0
#    (avoids routing ALL traffic over LTE — only broker traffic uses SIM7600)
ip route add <MQTT_BROKER_IP>/32 dev usb0
```

**Runtime LTE status check** (called from `sim7600_init()` in AntiDeathNode constructor):

```cpp
// Send AT commands over /dev/ttyUSB2 (SIM7600 AT port, separate from usb0 data port)
// Check network registration and signal quality before declaring ready.

struct SimStatus {
    bool  registered;   // AT+CREG? returns 1 (home) or 5 (roaming)
    int   rssi_dbm;     // AT+CSQ: signal strength
    bool  data_bearer;  // AT+CGACT? returns 1 (PDP context active)
};

SimStatus AntiDeathNode::query_sim_status() {
    // ... AT command exchange over /dev/ttyUSB2 ...
    // Log RCLCPP_WARN if rssi_dbm < -100 (weak signal — transmit may fail)
}
```

---

## 7. Paho MQTT Async Client Configuration

### 7.1 Client Initialization (Called at Node Startup)

```cpp
// anti_death_node.cpp — mqtt_client_init()

void AntiDeathNode::mqtt_client_init() {
    const std::string client_id = "vigia_" + device_id_.substr(0, 8);
    const std::string broker_uri =
        "ssl://" + mqtt_broker_host_ + ":" + std::to_string(mqtt_broker_port_);

    // ── Create async client ───────────────────────────────────────────────
    mqtt_client_ = std::make_unique<mqtt::async_client>(
        broker_uri,
        client_id,
        /*max_buffered_messages=*/5,  // small buffer — we send one large message
        nullptr                       // use default persistence (in-memory)
    );

    // ── TLS options (TLS 1.2 mutual authentication) ────────────────────────
    mqtt::ssl_options ssl_opts = mqtt::ssl_options_builder()
        // Server CA chain — validates broker identity
        .trust_store("/etc/vigia/ca_chain.pem")
        // Client certificate — proves device identity to broker
        .key_store("/etc/vigia/device_cert.pem")
        // Client private key — must match device_cert.pem Common Name
        .private_key("/etc/vigia/device_key.pem")
        // Force TLS 1.2 — SIM7600 firmware confirmed TLS 1.2 capable
        // MQTT_SSL_VERSION_TLS_1_2 maps to TLS_client_method() with min version set
        .ssl_version(MQTT_SSL_VERSION_TLS_1_2)
        // Verify broker hostname against certificate CN/SAN
        .verify(true)
        .finalize();

    // ── Connection options ────────────────────────────────────────────────
    conn_opts_ = mqtt::connect_options_builder()
        .ssl(ssl_opts)
        .keep_alive_interval(std::chrono::seconds(20))
        // Connection timeout per attempt — not the retry budget
        .connect_timeout(std::chrono::seconds(5))
        // Clean session = false: broker retains QoS 1 state across reconnects.
        // An inflight PUBLISH that didn't receive PUBACK before power cut will
        // be retransmitted on the next reboot+reconnect automatically.
        .clean_session(false)
        // Will message: broker publishes this if client disconnects uncleanly.
        // Server uses this to mark the device as offline in the fleet registry.
        .will(mqtt::will_options(
            "vigia/status/" + device_id_,
            "OFFLINE",
            /*qos=*/1,
            /*retained=*/true))
        .finalize();

    // ── Connect (blocking startup — not in the emergency window) ─────────
    auto tok = mqtt_client_->connect(conn_opts_);
    if (!tok->wait_for(std::chrono::seconds(15))) {
        // Non-fatal at startup: the client will retry in do_mqtt_connect().
        RCLCPP_WARN(get_logger(),
            "MQTT initial connect timed out. Will retry during emergency if needed.");
        return;
    }
    RCLCPP_INFO(get_logger(), "MQTT connected to %s (TLS 1.2 mutual auth).", broker_uri.c_str());

    // Publish online status (LWT inverse)
    mqtt_client_->publish(
        "vigia/status/" + device_id_, "ONLINE", 1, true)->wait();
}
```

### 7.2 MQTT Keepalive — Why Pre-Connection Works

The Paho async_client background thread sends MQTT `PINGREQ` every `keep_alive_interval` seconds and expects a `PINGRESP` from the broker. The LTE bearer stays active as long as there is periodic traffic. With a 20-second keepalive interval, the SIM7600's TCP connection remains open indefinitely.

During the emergency window, no keepalive overhead is incurred — MQTT keepalive timers are managed in Paho's background thread, which continues to run normally (SCHED_OTHER) alongside the SCHED_FIFO 99 emergency sequence. The emergency sequence calls `publish()` which uses the already-open TCP+TLS session — no handshake overhead.

**LTE bearer stability note:** If the Pi has been running for hours before a power failure, the LTE bearer may have experienced an idle timeout from the carrier. The `do_mqtt_connect()` fallback in §4.6 handles this case (broker reconnect ≤ 5 seconds in budget).

---

## 8. Server-Side Validation Contract

The VIGIA attestation server (or an edge validator feeding NVIDIA Cosmos 3) executes this exact verification pipeline for every received `vigia/attest/{device_id}/hazard` MQTT message.

### 8.1 MsgPack Decode and `EtHashInput` Reconstruction

```python
# server/attestation/validate_event.py

import msgpack
import struct
import hashlib
import numpy as np
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, utils as ec_utils
from cryptography.hazmat.backends import default_backend
from cryptography.exceptions import InvalidSignature
import database  # fleet registry (device_id → cert, last_sequence)

def validate_event(mqtt_payload_bytes: bytes) -> dict:
    """
    Full attestation pipeline for one VIGIA edge node event.
    Returns a validated dict on success; raises on any verification failure.
    """

    # ── Step 1: MsgPack decode ────────────────────────────────────────────
    payload = msgpack.unpackb(mqtt_payload_bytes, raw=False, strict_map_key=False)
    assert payload["version"] == 1, f"Unsupported payload version: {payload['version']}"

    device_id  = payload["device_id"]
    signed_et  = payload.get("signed_et")

    if signed_et is None:
        raise ValueError("Missing signed_et — unsigned payload rejected")

    # ── Step 2: Anti-replay (monotonic sequence check) ────────────────────
    # See §8.2.

    # ── Step 3: Reconstruct EtHashInput bytes for SHA-256 verification ────
    # The EtHashInput struct layout is IDENTICAL to the STM32 definition in
    # 03_stm32_firmware_contracts.md §6.3.2.
    # Layout (96 bytes, little-endian, explicit zero-padding):
    #   [16B] device_id (UUID bytes)
    #   [8B]  timestamp_us
    #   [4B]  sequence
    #   [16B] q_w, q_x, q_y, q_z  (float32 × 4)
    #   [12B] lin_accel_x/y/z      (float32 × 3)
    #   [1B]  imu_cal_status
    #   [3B]  _pad0 (must be 0x00)
    #   [16B] latitude, longitude   (float64 × 2)
    #   [4B]  altitude_m            (float32)
    #   [4B]  speed_ms              (float32)
    #   [4B]  course_deg            (float32)
    #   [1B]  fix_type
    #   [1B]  satellites
    #   [2B]  _pad1 (must be 0x00)
    #   [4B]  hdop                  (float32)
    # ──────────────────────────────────────────────────────────────────────

    # Parse device_id UUID bytes (strip hyphens from string UUID)
    device_uuid_bytes = bytes.fromhex(device_id.replace("-", ""))
    assert len(device_uuid_bytes) == 16

    et_input = struct.pack(
        "<"           # little-endian
        "16s"         # device_id[16]
        "Q"           # timestamp_us (uint64)
        "I"           # sequence (uint32)
        "ffff"        # q_w, q_x, q_y, q_z
        "fff"         # lin_accel_x, y, z
        "B3x"         # imu_cal_status + 3 zero-pad bytes
        "dd"          # latitude, longitude (float64 × 2)
        "fff"         # altitude_m, speed_ms, course_deg
        "BB2x"        # fix_type, satellites + 2 zero-pad bytes
        "f",          # hdop
        device_uuid_bytes,
        signed_et["stm32_timestamp_us"],
        signed_et["sequence"],
        # IMU fields reconstructed from signed_et metadata
        # (these were packed by the STM32 from BNO085 output)
        *extract_quaternion(signed_et),      # q_w, q_x, q_y, q_z
        *extract_lin_accel(signed_et),       # ax, ay, az
        signed_et.get("imu_cal_status", 0),
        # GPS fields
        payload["frame_metadata"][-1]["lat"],
        payload["frame_metadata"][-1]["lon"],
        payload["frame_metadata"][-1]["alt_m"],
        payload["frame_metadata"][-1]["speed_ms"],
        payload["frame_metadata"][-1].get("course_deg", 0.0),
        signed_et.get("gps_fix_type", 0),
        0,   # satellites — not separately stored in signed_et; use 0 for hash
        payload["frame_metadata"][-1].get("hdop", 99.9),
    )
    assert len(et_input) == 96, f"EtHashInput reconstruction error: {len(et_input)} bytes"

    return et_input, signed_et, payload
```

> **Implementation note for the server team:** The `struct.pack` format string above must be kept byte-for-byte identical to the `EtHashInput` struct in `03_stm32_firmware_contracts.md`. Any field reordering or type change in either location breaks ECDSA verification for ALL devices. This struct is the cryptographic contract between the STM32 firmware and the server — treat it with the same rigor as an ABI.

### 8.2 Anti-Replay Check

```python
def check_anti_replay(device_id: str, sequence: int) -> None:
    """
    Prevents replay attacks: an adversary capturing a valid signed packet and
    re-transmitting it to inject a false road hazard event.
    """
    # Fetch last known sequence from persistent fleet registry
    last_seq = database.get_last_sequence(device_id)

    if last_seq is not None and sequence <= last_seq:
        raise SecurityError(
            f"Replay attempt detected: device={device_id}, "
            f"received_seq={sequence}, last_seen_seq={last_seq}. "
            f"Packet rejected."
        )

    # Update registry — do this BEFORE ECDSA verify to prevent TOCTOU
    # (ECDSA is slow; an attacker flooding replays could exploit a verify-then-update gap)
    database.update_last_sequence(device_id, sequence)
    # Note: if ECDSA verify fails below, roll back this update.
    # Use a database transaction to make update + verify atomic:
    #   BEGIN TRANSACTION
    #   UPDATE last_sequence WHERE device_id = ? AND last_sequence < ?
    #   [verify ECDSA]
    #   COMMIT on success / ROLLBACK on failure
```

### 8.3 SHA-256 Reconstruction and Hash Verification

```python
def verify_et_hash(et_input_bytes: bytes, received_et_hash: bytes) -> None:
    """
    Verifies the et_hash field matches SHA-256(EtHashInput).
    The ATECC608A computed this hash internally via atcab_sha();
    we re-derive it from the struct to confirm the STM32 signed what it claimed.
    """
    computed_hash = hashlib.sha256(et_input_bytes).digest()

    if computed_hash != received_et_hash:
        raise IntegrityError(
            f"et_hash mismatch. "
            f"Received:  {received_et_hash.hex()}\n"
            f"Computed:  {computed_hash.hex()}\n"
            f"The ATECC608A signed a different payload than what was transmitted. "
            f"Possible STM32 firmware bug or data corruption in transit."
        )
```

### 8.4 ECDSA Signature Verification

```python
def verify_ecdsa_signature(
    device_id: str,
    et_hash: bytes,         # 32-byte SHA-256 digest — the signed message
    ecdsa_sig_raw: bytes    # 64-byte raw R∥S from ATECC608A
) -> None:
    """
    Verifies the ATECC608A ECDSA secp256r1 signature.

    ATECC608A atcab_sign() returns a 64-byte raw signature in IEEE P1363 format:
        [R: 32 bytes big-endian] ∥ [S: 32 bytes big-endian]

    Python's `cryptography` library requires DER-encoded ECDSA signatures.
    Convert by extracting R and S as integers and using encode_dss_signature().
    """
    assert len(ecdsa_sig_raw) == 64, "ECDSA signature must be 64 bytes (raw R∥S)"
    assert len(et_hash) == 32,       "et_hash must be 32 bytes (SHA-256 digest)"

    # ── Convert raw R∥S → DER ─────────────────────────────────────────────
    r = int.from_bytes(ecdsa_sig_raw[:32], byteorder='big')
    s = int.from_bytes(ecdsa_sig_raw[32:], byteorder='big')
    der_sig = ec_utils.encode_dss_signature(r, s)

    # ── Load device public key from fleet certificate registry ────────────
    device_cert_pem = database.get_device_cert(device_id)
    if device_cert_pem is None:
        raise AuthorizationError(
            f"Unknown device_id: {device_id}. "
            f"Device not provisioned in fleet registry. Rejected."
        )

    from cryptography.x509 import load_pem_x509_certificate
    cert = load_pem_x509_certificate(device_cert_pem.encode(), default_backend())
    public_key = cert.public_key()

    # ── Verify certificate chain ──────────────────────────────────────────
    # Validates device_cert against Microchip Trust Platform root CA.
    # Prevents forged certificates from passing verification.
    verify_certificate_chain(cert, database.get_root_ca())

    # ── Verify ECDSA signature ─────────────────────────────────────────────
    # We use Prehashed because the ATECC608A already computed SHA-256;
    # we do not want the Python library to double-hash.
    try:
        public_key.verify(
            der_sig,
            et_hash,                                 # the already-hashed digest
            ec.ECDSA(ec_utils.Prehashed(hashes.SHA256()))
        )
    except InvalidSignature:
        raise SecurityError(
            f"ECDSA verification FAILED for device={device_id}. "
            f"Signature does not match the public key in the device certificate. "
            f"Possible: firmware bug, key slot misconfiguration, or Sybil attack."
        )

    # Signature is valid — the ATECC608A holding device_id's private key
    # signed this exact EtHashInput at the recorded timestamp.
```

### 8.5 Spatial Latent Deserialization

```python
def deserialize_spatial_latent(latent_bin: bytes) -> np.ndarray:
    """
    Converts the msgpack bin field back to a float32 numpy array.
    Bytes are native LE float32 as packed by AntiDeathNode on ARM (Pi 5).
    """
    # '<f4' = little-endian 32-bit float
    latent = np.frombuffer(latent_bin, dtype='<f4')
    # latent.shape: (N,) — flat vector. Reshape to model's feature map dims if needed.
    return latent

def compute_scene_similarity(s_t: np.ndarray, reference_vectors: dict) -> dict:
    """
    Computes cosine similarity between S_t and stored reference scene vectors.
    Used by Cosmos 3 world model to validate semantic plausibility:
    if S_t is dissimilar to all known road scenes, flag as potential spoofed event.
    """
    similarities = {}
    s_t_norm = s_t / (np.linalg.norm(s_t) + 1e-8)
    for scene_class, ref_vec in reference_vectors.items():
        ref_norm = ref_vec / (np.linalg.norm(ref_vec) + 1e-8)
        similarities[scene_class] = float(np.dot(s_t_norm, ref_norm))
    return similarities
```

### 8.6 Complete Validation Pipeline and Cosmos 3 Submission

```python
def process_attestation_event(mqtt_payload_bytes: bytes) -> None:
    """
    Full pipeline: decode → anti-replay → hash verify → ECDSA verify → submit.
    Called by the MQTT subscriber for every message on vigia/attest/+/hazard.
    """
    try:
        # Parse and reconstruct
        et_input, signed_et, payload = validate_event(mqtt_payload_bytes)
        device_id = payload["device_id"]

        # Anti-replay (raises SecurityError on replay)
        check_anti_replay(device_id, signed_et["sequence"])

        # Hash integrity (raises IntegrityError on mismatch)
        verify_et_hash(et_input, bytes(signed_et["et_hash"]))

        # ECDSA signature (raises SecurityError on invalid sig)
        verify_ecdsa_signature(
            device_id,
            bytes(signed_et["et_hash"]),
            bytes(signed_et["ecdsa_sig"])
        )

        # All checks passed — construct Cosmos 3 world model update request
        latent = deserialize_spatial_latent(payload["spatial_latent"]) \
                 if payload.get("spatial_latent") else None

        cosmos3_payload = {
            "device_id":       device_id,
            "event_type":      "road_hazard_attested",
            "timestamp_us":    signed_et["stm32_timestamp_us"],
            "sequence":        signed_et["sequence"],
            "gps_location":    {
                "lat": payload["frame_metadata"][-1]["lat"],
                "lon": payload["frame_metadata"][-1]["lon"],
            },
            "rri_score":       payload.get("hazard_event", {}).get("rri_score"),
            "iss_score":       payload.get("hazard_event", {}).get("iss_score"),
            "spatial_latent":  latent.tolist() if latent is not None else None,
            "et_hash":         signed_et["et_hash"].hex(),
            "attestation_status": "VERIFIED",  # ECDSA + anti-replay passed
        }

        cosmos3_client.submit_world_model_update(cosmos3_payload)
        database.log_verified_event(device_id, cosmos3_payload)

    except (SecurityError, IntegrityError, AuthorizationError) as e:
        # Log to security audit trail — do NOT submit to Cosmos 3
        security_audit_log.record(
            event_type="ATTESTATION_REJECTED",
            device_id=payload.get("device_id", "UNKNOWN"),
            reason=str(e)
        )
        raise  # Propagate to MQTT subscriber for dead-letter queue handling
```

---

## 9. Anti-Death Node Memory Allocation Audit

| Operation | Allocation? | Notes |
|---|---|---|
| `SnapshotData snap{}` (stack) | **Stack** | ~37 KB on `vigia_antideath` stack. Ensure `ulimit -s unlimited` or increase `pthread` stack size to ≥ 256 KB at thread creation. |
| `msgpack::sbuffer buf` construction | **One-time heap** | The sanctioned allocation. `buf.reserve(500 KB)` triggers a single `malloc(500 KB)`. |
| `buf.pack_*(...)` calls (within capacity) | None | msgpack writes directly into the reserved buffer. No realloc if reserved size is sufficient. |
| `mqtt::make_message(...)` | **One-time heap** | Paho message object. Unavoidable — Paho owns message lifecycle through async I/O. |
| `msg->set_payload_ref(buf.data(), ...)` | None | Zero-copy reference into `buf`. |
| `mqtt_client_->publish(msg)->wait_for(...)` | None (after startup) | Paho internal I/O threads do allocate, but on the Paho-managed thread pool, not on `vigia_antideath`. |
| `on_spatial_latent()` callback (double-buffer) | None after warm-up | `vector::resize()` is no-alloc once capacity ≥ latent size (warm-up on 1st frame). |
| GPIO event struct | Stack | `gpiod_line_event`: 24 bytes, stack-allocated by libgpiod. |

**Stack size requirement:** `SnapshotData` is 37 KB on the stack (300 × 124 bytes). The default `pthread` stack size is 8 MB on Linux — ample. However, verify with `ulimit -s` and set explicitly in `launch_rt_node()`:

```cpp
// In launch_rt_node() for AntiDeathNode only:
pthread_attr_t attr;
pthread_attr_init(&attr);
pthread_attr_setstacksize(&attr, 512 * 1024);  // 512 KB — well above 37 KB + normal overhead
// Use attr in pthread_create() (or std::thread native_handle adaptation)
```

---

## 10. Systemd Unit Contract

```ini
# /etc/systemd/system/vigia-edge.service

[Unit]
Description=VIGIA ADAS DePIN Edge Node
After=network-online.target usb.service
Wants=network-online.target

[Service]
Type=simple
User=root                          # Required for SCHED_FIFO (CAP_SYS_NICE) and mlockall
ExecStartPre=/usr/local/bin/vigia-sim7600-init.sh  # AT commands → ECM mode → usb0 up
ExecStart=/opt/vigia/bin/vigia_edge_node \
    --ros-args \
    --params-file /etc/vigia/params.yaml
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

# Allow RT scheduling and memory locking (mandatory for SCHED_FIFO + mlockall)
LimitRTPRIO=99
LimitMEMLOCK=infinity
LimitNOFILE=65536

# Watchdog: if the process doesn't call sd_notify(WATCHDOG=1) every 30s,
# systemd kills and restarts it. AntiDeathNode sends WATCHDOG keepalive.
WatchdogSec=30
NotifyAccess=main

# Ensure /dev/shm is available and has sufficient space
MemorySwapMax=0                    # disable swap — shm must not be evicted

[Install]
WantedBy=multi-user.target
```

---

## 11. Acceptance Criteria

| Test | Method | Pass Condition |
|---|---|---|
| **Emergency sequence completes within 13s** | Inject UPS GPIO with `gpioset` + timestamp `journal` output | `SAFE_SHUTDOWN initiated` log appears ≤ 13.0s after GPIO assertion |
| **MQTT PUBACK received within budget** | Check `journal` for "PUBACK received" timestamp | T+PUBACK ≤ T+13.0s |
| **Seqlock snapshot zero torn reads** | Stress test: 60s of concurrent frame writes + 100 snapshots | All 100 snapshots have consistent `seq_start == seq_end` |
| **MsgPack payload size ≤ 500 KB** | Log `buf.size()` in `do_serialize()` | ≤ 512,000 bytes |
| **No realloc in sbuffer** | Instrument `msgpack::sbuffer::write()`: assert `size_ <= capacity_` throughout | Zero assertions fired |
| **TLS 1.2 mutual auth verified** | `openssl s_client -connect broker:8883 -cert device_cert.pem -key device_key.pem` | TLS session established, cipher suite is TLS\_ECDHE\_ECDSA\_WITH\_AES\_256\_GCM\_SHA384 or equivalent |
| **QoS 1 session persistence** | Kill vigia process mid-transmit; reboot; reconnect | Broker retransmits inflight PUBLISH on reconnect; server receives duplicate (handle via sequence dedup) |
| **ECDSA verify passes** | Server-side: `validate_event(captured_payload)` | No exceptions raised; `attestation_status = VERIFIED` |
| **Anti-replay rejected** | Replay same payload bytes to MQTT broker | Server raises `SecurityError`; event NOT submitted to Cosmos 3 |
| **EtHashInput struct size** | `python3 -c "import struct; print(struct.calcsize('<16sQIfffffff B3x ddfff BB2x f'))"` | `96` |
| **S_t cosine distance** | Capture S_t over 50 clear-road + 50 pothole frames; compute class means | Cosine distance between means ≥ 0.15 (validates S_t encodes scene semantics) |
| **SCHED_FIFO 99 confirmed** | `chrt -p $(pgrep -f vigia_antideath)` | `scheduling policy: SCHED_FIFO`, `priority: 99` |

---

## 12. Design Document Series — Complete

The VIGIA ADAS DePIN Edge Node specification is complete across five documents:

| Doc | Title | Status |
|---|---|---|
| `01_system_architecture_and_roadmap.md` | Master Architecture & 6-Phase Roadmap | APPROVED |
| `02_ros2_node_contracts.md` | ROS 2 Node Interface Contracts | APPROVED |
| `03_stm32_firmware_contracts.md` | STM32F411 Black Pill Firmware Contracts | APPROVED |
| `04_onnx_vision_engine_contracts.md` | ONNX Runtime Vision Engine Contracts | APPROVED |
| `05_anti_death_and_depin_contracts.md` | Anti-Death Storage & DePIN Attestation | **AWAITING APPROVAL** |

Upon approval of this document, the spec-driven design phase is complete. Phase 1 C++ implementation may begin with `02_ros2_node_contracts.md` as the primary reference (ROS 2 node scaffolding, `vigia_msgs` package, `ShmRingBuffer` implementation).
