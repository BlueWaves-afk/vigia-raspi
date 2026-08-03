# Riding the Blue Wave: Building Autonomous Intelligence on the Edge #03

*Episode 3: Designing for CPU-only Inference — squeezing real-time perception out of an ARM chip with no GPU.*

In the last episode I walked through the system architecture, the asynchronous multi-threaded pipeline that keeps the whole thing real-time. This episode goes one level deeper, into the part everyone assumes needs a GPU and where the Raspberry Pi flatly refuses to give you one: actually running the models.

There is no CUDA here. No tensor cores. Just a handful of ARM Cortex cores, a thermal budget, and two neural networks that both want to run right now. This is about what it actually takes to make that work.

## The constraint that shapes everything

A Pi 4 runs a Cortex-A72; the newer boards run an A76. Either way, the rule is the same: **inference happens on the CPU, and the CPU is also doing everything else** — capturing frames, analysing depth, fusing scores, drawing the dashboard. Every millisecond I spend on the model is a millisecond the rest of the system doesn't get.

So the first decision was to stop treating the two models the same. YOLO26n (object detection) has to run often, so it has to be cheap. MiDaS (depth estimation) is expensive but only needs to run occasionally. That split — established in the architecture — is what makes the precision decisions below possible.

## INT8 for YOLO, FP32 for MiDaS

The single biggest lever on a CPU is **quantization**. Running YOLO in full FP32 on an A72 is painful; running it in INT8 is a different game, because ARM has dedicated dot-product instructions (UDOT) that chew through 8-bit integer math far faster than float. So YOLO runs quantized to INT8, at roughly 83ms per frame — fast enough to run on essentially every frame.

MiDaS, on the other hand, stays FP32. Depth is a continuous, precision-sensitive output — quantizing it too aggressively wrecks the plane-fit geometry I rely on downstream — and since it only runs on every Nth frame anyway (the stride from Episode 2), its ~525ms cost is amortised. Different jobs, different precision. Forcing a single precision on both would have made one of them either too slow or too inaccurate.

## Why I moved off OpenVINO to ONNX Runtime + KleidiAI

The hackathon baseline used OpenVINO, and it worked, but it fought me on exactly the thing that mattered. It leaned on a forced FP32 hint that wasn't optimal for the ARM cores, and it was awkward to quantize cleanly for the INT8 UDOT fast path I wanted.

So I standardised on **ONNX Runtime (C++)** with the **ARM Compute Library + KleidiAI UDOT** execution provider as the INT8 fast path. This gets me portable, ARM-native INT8 inference that actually uses the hardware dot-product units. There's one important guard: the ACL UDOT provider is gated behind a CPUID check — I read `/proc/cpuinfo` for the `asimddp` flag before enabling it, because a chip without dot-product support would fall on its face. Detect the capability, don't assume it.

## The copies you don't see are the ones that cost you

Here is the part that surprised me most, and the reason I say the model is the easy 5%. On a GPU you barely think about memory traffic. On a CPU it dominates. When I profiled the preprocessing path, the model wasn't the bottleneck — the *plumbing around it* was:

- **Allocating fresh `cv::Mat` buffers every frame.** The preprocess chain allocated three intermediate images per frame, and the pixels were read and written three times before they ever reached the tensor. The fix was to pre-allocate every buffer once and reuse it, and — for the INT8 path — to wrap the input tensor's own memory as a `cv::Mat` so that padding writes *directly into the tensor*. Zero-copy, via ONNX Runtime's IO binding. The final memcpy simply disappears.
- **Doing the colour conversion before the resize.** `cvtColor` on a full 1280×720 frame touches 900k pixels; resize first and it only touches 100k. Swapping two lines was a ~9× reduction in that step's work.
- **Using `double` where `float` would do.** The plane-fitting loop accumulated in `double`, and the A72 has no vectorised double-precision path — NEON works on `float32x4`, not `float64`. Switching the accumulators to `float` let NEON auto-vectorise the loop for free.
- **A `medianBlur(5)` per detection.** Swapped for a `GaussianBlur(3)`, which is faster on ARM through the accelerated HAL.

None of these are clever. They are the kind of thing you would never bother with on a GPU, and every one of them is essential here. Constraints force clarity — you stop being sloppy about memory because the hardware won't forgive it.

## Closing Thoughts

The lesson of CPU-only inference is that the neural network is not where the performance lives. INT8 where you can afford the precision loss, FP32 where you can't, an inference runtime that actually speaks ARM's dialect, and then a relentless hunt for the allocations and copies hiding in the plumbing around the model. On a GPU those copies are noise. On a Cortex-A72 they *are* the frame budget.

In the next episode, I'll get into the concurrency itself — how to run capture, detection, and depth on different cores without the whole thing collapsing into a tangle of locks and race conditions. Asynchronous pipelines, without the chaos.

*our [github repo](https://github.com/BlueWaves-afk/vigia-raspi).*
