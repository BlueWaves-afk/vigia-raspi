# Riding the Blue Wave: Building Autonomous Intelligence on the Edge #04

*Episode 4: Asynchronous Pipelines Without Chaos — threads, cores, lock-free buffers, and the day I caught my own "parallel" pipeline running sequentially.*

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
