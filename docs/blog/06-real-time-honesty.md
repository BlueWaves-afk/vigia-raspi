# Riding the Blue Wave: Building Autonomous Intelligence on the Edge #06

*Episode 6: Why Most "Real-Time" Demos Aren't Actually Real-Time — a confession about my own benchmarks, and how to profile honestly on ARM.*

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
