I'll start with the confession. My README said the system ran at 10.3 FPS with a real-time parallel pipeline. When I sat down and actually measured it running with the display on, it managed **2–3 FPS**, and its worst-case frame latency was **608 milliseconds**. Both the 10.3 and the 2–3 were "true" numbers. That gap — between the number you quote and the number the system actually delivers when it matters — is what this final episode is about.

## What "real-time" actually has to mean

"Real-time" is one of the most abused words in edge AI. It usually means "the average frame rate looked good in the config where nothing was fighting for the CPU." That is not real-time. For a system meant to flag a pothole in front of a moving vehicle, real-time means one specific thing:

**The 95th-percentile latency fits your budget — on the worst frame, in the real deployment config.**

Not the average. Not the median. The tail. Because the frame that matters is the one where the expensive model, the thermal throttle, and the display all land at once, and if *that* frame takes 600ms, it doesn't matter that your average was a pretty 100ms. You saw the hazard half a second late.

## Where my FPS actually went

When I stopped trusting the average and profiled the whole system, almost none of the loss was in the models. YOLO at 83ms and strided MiDaS were fine. The FPS was being eaten by the surrounding system:

- **`cv::imshow` + VNC encoding cost me 3.4×.** Displaying the dashboard over VNC dropped throughput from 10.3 FPS to ~3, because VNC encodes the framebuffer *on the same CPU cores that run inference*. My "benchmark" was headless; my "demo" had the display on. Those are two completely different machines, and I'd been quoting one while showing the other.
- **A frame-limiter bug throttled me from the first frame.** I passed a target of 30 FPS into a governor that also controlled the depth stride. Since real throughput was ~10 FPS, the governor decided it was permanently "overloaded" and slammed MiDaS to its most aggressive stride immediately — even on a cold, idle chip. A configuration mismatch was silently degrading depth quality, and the average FPS hid it completely.
- **The sequential-not-parallel pipeline** from Episode 4, which put a 608ms spike on every depth frame.
- **Per-frame allocations and a `std::cout` in the hot path**, each individually small, collectively a tax on every single frame.

Not one of those is a model problem. They are all systems problems, and the average frame rate was actively concealing every one of them.

## Averages lie; percentiles and configs don't

Two habits fixed my benchmarking honesty.

First, **stop reporting the mean; report the distribution.** An exponential moving average of 10.3 FPS and a P95 latency of 608ms describe the same system, and only the second one tells you whether it's safe. The mean is what a system does when it's comfortable. The P95 is what it does when it's under pressure — which, for a safety system, is the only number that counts.

Second, **always state the config.** Headless or display? What input resolution? Thermals cool or throttling? "10.3 FPS" is meaningless without "headless, 1280×720, cool." The moment I started writing the config next to every number, the contradictions in my own claims became obvious. A benchmark without its conditions attached isn't a benchmark; it's marketing.

## Observability is how you catch a fake

I only found all of this because I'd built the system to be *watchable*. There's a visual test harness that replays a fixed clip (`hazard.mp4`) deterministically, so I can compare runs apples-to-apples, and an instrumentation layer that records timings into a lock-free ring buffer and handles depth results arriving out of order. Being able to *see* the pipeline — which stage ran when, which frames dropped, where the 600ms went — is what turned "it feels slow" into "here are the eight specific reasons, ranked by impact."

This is the part most demos skip, and it's why most demos can get away with claiming real-time: without instrumentation, nobody — including the author — actually knows. Visual debugging and honest telemetry aren't a nice-to-have on an autonomous system. They are the only way you find out that your real-time system isn't.

## Closing Thoughts — and the end of the series

If there's one idea to take from this whole series, it's the one this episode makes concrete. Real-world edge AI is a systems-engineering problem, not a model problem — and "real-time" is a claim you have to *earn* with measurement, under the real config, at the tail of the distribution, not the average. The model was maybe 5% of this project. The threading, the memory discipline, the thermal governor, the temporal reasoning, the instrumentation, and the honesty about what the numbers actually mean were the other 95%.

That's the through-line from Episode 1's motivation to here: constraints don't just make the engineering harder, they make it *honest*. A Cortex-A72 with no GPU and a thermal ceiling won't let you hide behind a good average. It forces you to build a system that survives its own worst frame — and to be truthful about whether it does.

Thanks for riding the wave with me. Road hazard detection was just the story; the real journey was learning to build autonomous intelligence that survives constraints, and to measure it without lying to yourself. ❤️

*our [github repo](https://github.com/BlueWaves-afk/vigia-raspi).*

---

## 🎓 CS Fundamentals — study companion

*This finale is about **real-time systems**, **performance measurement / statistics**, and the **System Design** discipline of observability and honest benchmarking — plus a **Computer Architecture** note on why the display stole your FPS. These "how do you measure it" questions separate senior candidates from junior ones.*

### Operating Systems (OS) — real-time systems

**What this post touches:** hard vs soft real-time, latency vs throughput, tail latency, resource contention, scheduling.

**Deep dive.**
- **Hard vs soft real-time.** *Hard* real-time: missing a deadline is a system failure (airbags, motor control). *Soft* real-time: a missed deadline degrades quality but is tolerable (video playback). A hazard detector is *firm/soft* — a late alert is nearly useless but not catastrophic. The correctness criterion is **latency**, not just throughput: producing the right answer *too late* is a wrong answer.
- **Throughput vs latency — different metrics.** FPS is *throughput* (frames/sec). Per-frame **latency** is time from capture to decision. A pipeline can have high throughput and terrible worst-case latency (the sequential-MiDaS 608ms spike). For safety, latency — specifically the **tail** — is what counts.
- **Resource contention (the VNC lesson).** `cv::imshow` over VNC encodes the framebuffer **on the same CPU cores** running inference, so the display *competes* with inference for compute — dropping 10 FPS to 3. This is contention for a shared resource (CPU), the same class of problem as lock contention but for compute cycles. **Lesson:** benchmark the *deployment* config, because a "harmless" component can steal the resource your hot path needs.

**Interview Q&A.**
1. *Hard vs soft real-time — example of each?* → Airbag (hard) vs video streaming (soft); define by the cost of a missed deadline.
2. *Throughput vs latency — can you have one without the other?* → Yes: batching raises throughput but can raise latency; a pipeline can average fast yet have a huge P95 tail.
3. *Why did turning on the display tank inference FPS?* → CPU contention — VNC encodes on the same cores; the display and inference fight for compute.

### Performance measurement & Statistics (shows up in SDE + SRE interviews)

**Deep dive.**
- **Averages lie; use percentiles.** The mean hides the tail. **P95/P99 latency** = the value 95%/99% of requests come in under. A 100ms mean with a 608ms P95 is a system that's usually fine and periodically dangerous. For anything user- or safety-facing, you design and report against the tail, not the mean. (This is why big-tech SLOs are written as "P99 < X ms.")
- **Why the tail dominates.** In a fast-moving car, the frame that matters is the worst one — where the expensive model, thermal throttle, and display all coincide. Optimising the average while ignoring the P95 optimises the wrong thing.
- **EMA vs instantaneous.** An **Exponential Moving Average** (`ema = α·x + (1−α)·ema`) smooths a noisy metric to show a trend, but it *also* smooths away spikes — so an EMA FPS looks stable even when individual frames stall. Report both the smoothed trend *and* the distribution.
- **Benchmark methodology / reproducibility.** A number without its config (headless vs display, input resolution, thermal state, warm vs cold cache) is meaningless. The blog's headless-10.3 vs display-3 contradiction is a **methodology** failure, not a code failure. Always: fix the input (deterministic replay of `hazard.mp4`), state the environment, run enough samples, report the distribution.

**Interview Q&A.**
1. *Why report P99 instead of the average latency?* → The average hides the tail that users/safety actually feel; SLOs target the tail.
2. *What's an EMA and what does it hide?* → `α·x+(1−α)·ema`; smooths trend but masks spikes — pair it with percentiles.
3. *How would you benchmark this system credibly?* → Deterministic input, stated config, warm-up, many samples, report P50/P95/P99 + throughput, not a single mean.
4. *Your average latency is fine but users complain. What do you check?* → The tail (P95/P99), GC/allocation pauses, contention, cold-cache first requests, variance.

### System Design — observability & SLOs
- **Observability is how you catch a lie.** The instrumentation layer (lock-free telemetry ring, deterministic replay, out-of-order depth handling) is what turned "feels slow" into "eight ranked, specific causes." **You cannot optimise what you cannot measure**, and you cannot claim "real-time" without measuring the tail under load. This is the metrics/logging/tracing pillar of system design (the three pillars of observability: **metrics, logs, traces**).
- **SLI / SLO / SLA.** An **SLI** is the measured indicator (e.g., P95 frame latency); an **SLO** is the target (P95 < 150ms); an **SLA** is the contractual promise. "Real-time" should be written as an SLO on the tail, then verified.
- **Instrumentation must be cheap.** Logging in the hot path (the `std::cout` from Ep 4) *changes* what you measure (observer effect) — hence the lock-free ring-buffer sink. Good telemetry is low-overhead and off the critical path.

**Interview Q&A.**
1. *What are the three pillars of observability?* → Metrics, logs, traces.
2. *Define SLI/SLO/SLA.* → Indicator / target / contract.
3. *How do you measure a system without slowing it down?* → Low-overhead, off-hot-path telemetry (ring buffers, sampling), async export; beware the observer effect.

### Computer Architecture — cameo
- **Shared-core contention** is the compute analogue of cache/bus contention: two jobs on the same cores serialise on the ALUs. The fix mirrors the OS one — isolate the heavy job (dedicated core / headless), or shed the optional work (lower render rate).

### Quick-review flashcards
- **Hard vs soft real-time**; correctness = *latency*, not just throughput.
- **Throughput (FPS) ≠ latency (per-frame); the tail (P95/P99) is what matters.**
- **Averages lie → report percentiles.**
- **EMA** smooths trend *and* hides spikes.
- **Benchmark = deterministic input + stated config + distribution.**
- **Observability pillars:** metrics, logs, traces. **SLI/SLO/SLA.**
- **Observer effect:** measurement must be cheap and off the hot path.
