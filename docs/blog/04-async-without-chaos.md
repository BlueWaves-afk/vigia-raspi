Adding threads to a system is easy. Adding threads *without* turning it into a minefield of race conditions, priority inversions, and mysterious frame drops is the actual engineering. In Episode 2 I described the pipeline as asynchronous and multi-threaded. This episode is about the discipline that keeps "asynchronous" from becoming "unpredictable" — and about a bug that taught me to distrust my own architecture diagrams.

## Threads are cheap; shared state is expensive

The pipeline runs on several threads — capture, process (YOLO), depth (MiDaS), output — and the first rule I hold to is that **threads don't cause bugs, shared state does.** Two threads that never touch the same memory can run as fast as they like. The trouble starts the moment they share a buffer.

So each thread is pinned to its own core (`pinToCore`), and the boundaries between them are as small and as explicit as I can make them. The capture thread owns the camera. The process thread owns YOLO. The depth thread owns MiDaS. Where they must hand data across, they do it through exactly one narrow, well-understood channel — not by casually reaching into each other's state.

## Latest-wins where staleness is fine, queues where it isn't

Not every hand-off wants the same data structure, and picking the wrong one is how you get lag.

Between **capture and process**, the right structure is a ring buffer with latest-wins semantics. The camera writes frames continuously; the process thread always reads the *newest* one and lets stale frames get overwritten. In a fast-moving vehicle you would always rather process the freshest frame than work through a backlog — a queue here would just accumulate latency you can never pay back.

Between **process and depth**, the right structure is a work queue (a `SafeQueue`). The process thread pushes a frame for MiDaS and *moves on immediately* — it never waits for depth. The depth thread drains the queue at its own slower pace. This is the decoupling that lets a 525ms model live inside a system that has to stay real-time: the fast thread is never allowed to block on the slow one.

## The bug: clone-under-lock

Here's a concrete one that cost real FPS. The capture thread was cloning each frame *while holding the buffer mutex*:

```cpp
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frameBuffer_[i] = frame.clone();   // ~0.9ms, all inside the lock
}
```

A 640×480 clone is under a millisecond, but it held the mutex for that entire millisecond, blocking the process thread from reading the latest frame. The fix is trivial once you see it — clone *outside* the lock, then only the pointer swap happens inside:

```cpp
cv::Mat cloned = frame.clone();        // outside the lock
{
    std::lock_guard<std::mutex> lock(bufferMutex_);
    frameBuffer_[i] = std::move(cloned);  // just a move, microseconds
}
```

The lesson generalises: whatever you do inside a lock is time every other thread spends waiting. Do the expensive work outside, and hold the lock only for the handful of instructions that genuinely need to be atomic. (The same instinct killed a `std::cout` I'd left in the hot path — it was grabbing a global lock on every detection.)

## The uncomfortable discovery: "parallel" that wasn't

Now the humbling part. My README proudly described a "4-stage parallel pipeline." When I actually profiled the frame latency, I found YOLO and MiDaS were running **sequentially on the same core**:

```cpp
auto detections = perception_.runInference(frame);   // ~83ms
if (currentIdx % midasStride_ == 0)
    depthMap = analytical_.runInference(frame);       // ~525ms
// worst case: 83 + 525 = 608ms on one thread
```

The system *looked* fast because MiDaS only ran at stride 5, so four out of five frames skipped the expensive call entirely and the average looked great. But on the frame where MiDaS did run, latency spiked to 608ms — and 608ms in a moving car is a hazard you see far too late. The architecture diagram said parallel; the code said sequential. The profiler doesn't care what your diagram says.

The fix was to make it actually parallel: MiDaS moved to its own dedicated thread on Core 2, fed by the work queue, posting depth results asynchronously when ready. Now YOLO's 83ms *is* the frame latency, even on depth frames. The 608ms tail disappeared.

## Where priorities cross, drop the mutex entirely

There's one more layer for the paths that genuinely can't tolerate a stall. For the highest-priority hand-offs — a safety monitor snapshotting the frame ring while the camera is mid-write — even a well-behaved mutex is dangerous, because a high-priority thread blocking on a lock held by a lower-priority one is textbook priority inversion. The answer there is a **seqlock**: the writer brackets each write with an atomic counter (odd = writing, even = stable), and the reader spins until it sees a stable, consistent snapshot, then re-checks. No lock is ever taken, so there's nothing to invert. Paired with a real-time scheduling policy and a clear priority ladder across the threads, the critical path stays wait-free.

One subtlety that falls out of true async: the frame ring has to be big enough that a slot isn't overwritten while MiDaS is still reading it. With MiDaS taking ~525ms and capture at 15 FPS, the buffer needs about eight slots (≈533ms) of headroom. Get that wrong and your "parallel" depth thread quietly reads corrupted frames.

## Closing Thoughts

Asynchronous done right is not "sprinkle threads and hope." It is deliberate ownership: one owner per buffer, latest-wins where freshness beats completeness, queues where you must decouple a slow stage, expensive work kept outside every lock, and lock-free structures on the paths where priorities cross. And it is the humility to profile — because the day I actually measured it, my beautiful parallel pipeline turned out to be a sequential one wearing a costume.

In the next episode, I'll get into temporal reasoning — why a slightly weaker detection that shows up consistently is worth more than a confident one that flashes for a single frame.

*our [github repo](https://github.com/BlueWaves-afk/vigia-raspi).*

---

## 🎓 CS Fundamentals — study companion

*This is the **Operating Systems** episode — concurrency, locks, priority inversion, and real-time scheduling — plus the **Data Structures** behind lock-free buffers. If you master this one you can answer almost any concurrency question, which are among the most common in interviews.*

### Operating Systems (OS) — concurrency & synchronization

**What this post touches:** race conditions, critical sections, mutexes, lock contention, priority inversion & inheritance, real-time scheduling (SCHED_FIFO), CPU affinity, deadlock.

**Deep dive.**
- **Race condition & critical section.** A race is when the result depends on thread timing over shared state. The fix is to make the update a **critical section** — a region only one thread executes at a time — guarded by a lock. The frame buffer is shared state; without a lock, capture and process can tear a frame.
- **Mutex & the cost of the critical section.** A mutex serialises access. The crucial lesson of the "clone-under-lock" bug: **whatever runs inside the lock is time every other thread waits.** Holding the mutex during a ~1ms `frame.clone()` blocked the reader for that whole millisecond. Fix = shrink the critical section to a pointer swap; do the expensive work outside. General principle: *minimise the critical section.*
- **Lock contention & the hot-path `std::cout`.** `std::cout` takes a global lock; calling it per detection serialises threads on I/O. Contention is invisible until you profile — the reason logging belongs in a lock-free ring, not the hot path.
- **Priority inversion — the classic RT bug.** A high-priority thread H waits on a lock held by a low-priority thread L; a medium-priority thread M preempts L, so L never releases, so H is stuck behind M forever. This actually killed the Mars Pathfinder mission. **Fixes:** *priority inheritance* (L temporarily inherits H's priority so it finishes and releases) or avoiding the shared lock entirely. The blog's answer for the critical path is the latter — a lock-free **seqlock**.
- **Real-time scheduling.** A **real-time** thread needs *bounded* latency, not just speed. `SCHED_FIFO` (a fixed-priority, run-to-completion policy) + a **PREEMPT_RT** kernel + a **priority ladder** (safety monitor > sensors > camera > vision > fusion) ensures the important thread runs when it must. Contrast with the default fair scheduler (CFS), which optimises average throughput, not worst-case latency.
- **CPU affinity.** Pinning each stage to its own core (`pinToCore`) gives cache warmth and stops the scheduler from migrating a hot thread. Combined with RT priorities, it makes latency predictable.
- **Deadlock (bonus).** Four Coffman conditions: mutual exclusion, hold-and-wait, no preemption, circular wait. Avoid by lock ordering, try-lock, or (best here) not sharing a lock at all.

**Interview Q&A.**
1. *What is a race condition and how do you prevent it?* → Timing-dependent bug on shared state; guard the critical section with a lock (or use atomics/lock-free structures).
2. *Why should critical sections be short?* → The lock holder blocks everyone else; long critical sections = contention and latency. (This is the single most reusable takeaway.)
3. *Explain priority inversion and priority inheritance.* → H blocked by L, L preempted by M; inheritance bumps L to H's priority to release the lock. Cite Mars Pathfinder.
4. *SCHED_FIFO vs CFS?* → Fixed-priority run-to-completion (bounded latency, real-time) vs fair time-sharing (throughput/fairness).
5. *What are the four conditions for deadlock?* → Mutual exclusion, hold-and-wait, no preemption, circular wait; break any one.
6. *A high-priority thread must snapshot data a lower-priority thread is writing, without ever blocking. How?* → A lock-free seqlock (below), avoiding the mutex that would invert priorities.

### Data Structures & Algorithms (DSA) — lock-free & concurrent structures

**Deep dive.**
- **Ring buffer as a concurrency tool.** A single-producer/single-consumer ring can be made **lock-free** with atomic head/tail indices and correct memory ordering — no mutex at all. Great for the capture→process hand-off.
- **Seqlock (sequence lock).** For one writer, many readers, where reads must never block writers: the writer increments a counter before (→ odd = "writing") and after (→ even = "stable") each write. A reader records the counter, copies, then re-reads the counter; if it changed or is odd, it retries. Readers are **wait-free-ish** (they may retry but never hold a lock), and the high-priority reader can never invert the writer's priority. Trade-off: readers can starve under a torrent of writes, and it only works when a retry (re-copy) is cheap.
- **Concurrent queue (SafeQueue).** A thread-safe queue (mutex + condition variable, or lock-free) that lets `push` return immediately while a consumer `wait_and_pop`s. This is the producer–consumer bounded-buffer, realised.
- **Memory ordering / atomics (COA crossover).** Lock-free code needs `std::atomic` with the right memory order (acquire/release) so the compiler/CPU don't reorder the "data write" past the "flag write." This is why lock-free is subtle: correctness depends on the hardware memory model.
- **Buffer sizing = Little's Law.** The frame ring must hold ≥ (MiDaS latency × capture rate) frames so a slot isn't overwritten mid-read: ~525ms × 15 FPS ≈ 8 slots. That's **Little's Law** (`L = λ·W`): items in the system = arrival rate × time-in-system.

**Interview Q&A.**
1. *How do you make a single-producer/single-consumer queue lock-free?* → Atomic head/tail with acquire/release ordering; no mutex.
2. *What's a seqlock and when do you use it?* → Wait-free reads for one writer / many readers where retry is cheap; avoids priority inversion.
3. *How big should a buffer between a fast producer and slow consumer be?* → Little's Law: cover the consumer's latency window at the producer's rate; add margin.
4. *Why do atomics need memory ordering?* → To stop compiler/CPU reordering that would let readers see a "ready" flag before the data it guards.

### System Design
- **The "measure, don't trust the diagram" lesson.** The architecture *claimed* parallel; profiling showed sequential (608ms P95). **Observability beats assumption** — a recurring system-design theme (revisited in Episode 6).
- **Decoupling via async stages.** Never let the fast stage block on the slow one; hand off through a queue and let each run at its own rate.

### Quick-review flashcards
- **Critical section:** keep it *tiny* (the clone-under-lock lesson).
- **Priority inversion:** H→L→M; fix with inheritance or lock-free. (Mars Pathfinder.)
- **SCHED_FIFO + PREEMPT_RT + affinity** → bounded latency.
- **Seqlock:** odd/even counter, reader retries, wait-free, no inversion.
- **Deadlock:** 4 Coffman conditions.
- **Little's Law:** `L = λ·W` → buffer sizing.
- **Lock-free needs atomics + acquire/release ordering.**

### ⚖️ This vs That — the architecture decisions, and the roads not taken

| Decision | Alternatives | Why this choice |
|---|---|---|
| **Seqlock (lock-free) on the RT path** | `std::mutex`; RCU; spinlock | A mutex on a high-priority reader invites priority inversion; RCU is powerful but complex and grace-period-heavy. A seqlock is simple, wait-free for one-writer/many-reader, and never inverts priority — perfect when a retry (re-copy) is cheap. |
| **Explicit `std::thread` + core pinning** | Thread pool; async runtime / coroutines; OpenMP | Pools and runtimes abstract away exactly what a real-time system needs: which thread runs on which core at which priority. Explicit threads give that control. |
| **Message-passing queues (where decoupling)** | Shared memory + locks everywhere | Queues make ownership and backpressure explicit and avoid shared-state races. Shared memory is used *only* where latest-wins semantics make it simpler (the frame ring). |
| **SCHED_FIFO + PREEMPT_RT** | Default CFS scheduler | CFS optimises fairness and average throughput; it makes no worst-case latency guarantee. Real-time work needs fixed priorities and a preemptible kernel so the critical thread runs *when* it must. |
| **Dedicated MiDaS thread (Core 2)** | Keep MiDaS inline (the original bug) | Inline made "parallel" actually sequential (608ms P95). A dedicated thread makes YOLO's 83ms the frame latency even on depth frames. |

**The one to defend:** *lock-free vs mutex on the hot path.* Most candidates reach for a mutex reflexively. The stronger answer names the failure mode — **priority inversion** — and picks the tool to the access pattern: mutex for general mutual exclusion, but a **seqlock/lock-free** structure when a high-priority thread must never block on a low-priority one. Knowing *when a mutex is the wrong tool* is the senior signal.
