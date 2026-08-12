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

---

## 🧰 The inference stack, from zero — quantization, SIMD, and what I chose it over

Making a neural net run on a bare ARM CPU is mostly about *moving fewer bytes and doing more per instruction*. From zero:

- **INT8 quantization (over FP32).** Store weights/activations as 8-bit integers instead of 32-bit floats: ~4× less memory traffic and access to integer dot-product hardware. The cost is a small accuracy hit, managed with **calibration**. On a memory-bound device this is the single biggest win — the cores were starving for data, not arithmetic.
- **SIMD / NEON + UDOT (the hardware basis).** SIMD = one instruction over a vector of values. ARM's **UDOT** dot-product instruction (FEAT_DotProd) multiply-accumulates a whole INT8 vector in one go — exactly a matmul's inner loop. It's gated behind a `/proc/cpuinfo asimddp` CPUID check so the binary degrades on chips without it.
- **Execution providers (ONNX Runtime) — over OpenVINO and TFLite.** ONNX Runtime has pluggable backends; the **ARM Compute Library / KleidiAI** provider is the INT8 fast path. We moved off the **OpenVINO** hackathon baseline (forced FP32, awkward to quantise) and chose ONNX over **TFLite** for C++ ergonomics and the ARM INT8 story. Honest status: migration in progress; the current build still links OpenVINO.
- **Zero-copy IO-binding.** Pre-bind input/output tensors once so there's no per-frame `malloc`/copy on the hot loop. In a memory-bound system, eliminating data movement *is* the optimization.

## 🚢 From demo to production

- **CPU feature-gating** — probe `asimddp` before using the INT8 path; fall back gracefully.
- **Model versioning + warmup** — pin the model artifact, run a warmup inference so the first real frame isn't slow.
- **Thermal-aware scheduling** — the chip throttles under load, so cold-start benchmarks lie; budget for the hot steady state (Episode 6).

---

## 🎓 CS Fundamentals — study companion

*This is the **Computer Architecture** episode. Quantization, SIMD, the memory hierarchy, and zero-copy are exam favourites and this post is built entirely from them. There's also solid **OS** (memory allocation) and **System Design** (capability detection) material.*

### Computer Organization & Architecture (COA)

**What this post touches:** number formats & quantization (INT8/FP32), SIMD/vectorization (NEON, UDOT), the memory hierarchy & cache, data locality, zero-copy, CPUID/feature detection.

**Deep dive.**
- **Number formats.** FP32 = 32-bit float (1 sign, 8 exponent, 23 mantissa) — wide dynamic range, expensive. INT8 = 8-bit integer — 4× less memory, and integer ALUs are far cheaper/faster. **Quantization** maps real weights/activations to INT8 via a scale (and zero-point): `real ≈ scale × (q − zero_point)`. You trade precision for speed+size. The blog quantizes **YOLO to INT8** (throughput matters, small accuracy loss is fine) but keeps **MiDaS in FP32** (depth is precision-sensitive). Classic engineering: match precision to the job.
- **SIMD & NEON.** **SIMD** = Single Instruction, Multiple Data: one instruction operates on a vector of values at once. ARM's SIMD unit is **NEON**, which has `float32x4_t` (4 floats/op) but *not* a useful 64-bit-double vector path — which is exactly why the blog's `double`→`float` fix matters: doubles run **scalar** (one at a time), floats run **vectorised** (four at a time). **UDOT** is a NEON instruction that does an 8-bit dot-product in one shot — the hardware reason INT8 inference flies on ARM.
- **The memory hierarchy.** Registers → L1 → L2 → (L3) → RAM → disk, each ~10× larger and slower than the last. A Cortex-A72 L1 hit is ~1–4 cycles; a RAM access is ~100+ cycles. So **cache misses**, not arithmetic, dominate. Every "pre-allocate the buffer and reuse it" fix in the post is really "stop thrashing the cache/allocator."
- **Data locality & why allocation is expensive.** Allocating a fresh `cv::Mat` every frame (a) calls the heap allocator (a synchronization + bookkeeping cost) and (b) touches cold memory (cache misses + page faults). Reusing a buffer keeps the data **hot** in cache. The "resize before cvtColor" fix is a locality win too: do the expensive per-pixel op on 100k pixels, not 900k.
- **Zero-copy.** The most expensive thing you can do with data is copy it. By wrapping the model's input tensor memory as a `cv::Mat` and writing preprocessing *directly into it*, the code removes an entire 300KB memcpy per frame. Zero-copy = share a pointer instead of duplicating bytes. (Same idea as `sendfile()`, `mmap`, and DMA in OS/networking.)
- **CPUID / capability detection.** Not every ARM chip has the dot-product extension. The code reads `/proc/cpuinfo` for the `asimddp` flag before enabling the UDOT fast path. This is **runtime ISA feature detection** — the same reason x86 code checks `CPUID` for AVX before using it. *Detect, don't assume.*

**Interview Q&A.**
1. *What is quantization and what does it cost?* → Mapping FP32 → lower-bit (e.g., INT8) via scale/zero-point; 4× smaller + faster integer math; costs some precision (mitigated by calibration / quantization-aware training).
2. *What is SIMD? Give an example.* → One instruction on a vector; e.g., NEON `float32x4_t` adds 4 floats at once. Why `double` is slower on ARM: no vectorised double path → scalar execution.
3. *Explain the memory hierarchy and why cache misses matter.* → Speed/size tradeoff across registers→cache→RAM→disk; a miss costs ~100 cycles, so locality dominates real performance.
4. *What is zero-copy and why does it help?* → Avoid duplicating data by sharing a pointer/mapping; saves bandwidth and cache pressure; examples: `mmap`, `sendfile`, DMA, wrapping a tensor buffer.
5. *How do you use a CPU feature that only some chips have?* → Runtime detection (CPUID / `/proc/cpuinfo`) + a fallback path.
6. *Why do per-frame heap allocations hurt on an edge CPU?* → Allocator overhead + cache-cold memory + fragmentation + (on constrained devices) page faults; fix by pre-allocating and reusing.

### Operating Systems (OS)

- **Heap vs stack allocation.** Stack allocation is a pointer bump (nearly free); heap allocation (`malloc`/`new`) walks free lists, may lock, and can fragment. The hot loop avoids heap churn by hoisting buffers out of the loop. **The general rule:** no allocation in the steady-state hot path.
- **Virtual memory & page faults.** First touch of freshly-allocated memory triggers a page fault (kernel maps a physical page). Reusing warm buffers avoids repeated faults — relevant on a memory-pressured Pi with SD-card swap.

**Interview Q&A.**
1. *Stack vs heap allocation cost?* → Bump-pointer vs allocator bookkeeping/locking/fragmentation.
2. *What is a page fault?* → Trap when accessing an unmapped/mapped-but-not-resident page; the kernel loads/maps a page. Minor vs major faults.

### System Design
- **Portability via an abstraction + fast-path.** Standardising on ONNX Runtime (portable) with an ACL/KleidiAI INT8 execution provider (fast, ARM-native) behind a capability gate is a clean pattern: *a portable interface with a hardware-accelerated fast path selected at runtime.*

### Quick-review flashcards
- **Quantization:** `real ≈ scale·(q − zero)`; INT8 = 4× smaller, integer-fast.
- **SIMD/NEON:** one instr, many data; `float32x4` vectorises, `double` doesn't on A72.
- **UDOT:** 1-instruction INT8 dot product → why INT8 flies on ARM.
- **Memory hierarchy:** miss ≈ 100 cycles → locality wins.
- **Zero-copy:** share a pointer, don't memcpy.
- **CPUID / `asimddp`:** detect the feature, keep a fallback.
- **Hot-path rule:** zero heap allocation in steady state.

### ⚖️ This vs That — the architecture decisions, and the roads not taken

| Decision | Alternatives | Why this choice |
|---|---|---|
| **ONNX Runtime + ACL/KleidiAI (INT8 UDOT)** | OpenVINO; TFLite; ncnn; raw NEON kernels | OpenVINO forced FP32 and fought INT8 quantization; TFLite/ncnn are strong on ARM but ONNX Runtime gave the best mix of *portability* + a native INT8 dot-product fast path + zero-copy IO binding. Raw NEON is fastest but unmaintainable. |
| **INT8 for YOLO** | FP16; FP32 | The A72 has no broadly useful FP16 *compute* path, and FP32 is slow. INT8 hits the UDOT hardware — the only real speedup available. |
| **FP32 for MiDaS** | INT8 depth | Depth feeds precision-sensitive plane geometry; aggressive quantization wrecks it. Since MiDaS is strided, its FP32 cost is amortised — so keep the accuracy. |
| **Zero-copy IO binding** | memcpy preprocessing → tensor | Copying 300KB/frame is pure waste on a bandwidth-bound CPU. Writing preprocessing directly into the tensor buffer removes the copy entirely. |
| **Compiler auto-vectorization (float)** | Hand-written NEON intrinsics | Intrinsics are ~10% faster but brittle and unportable. Feeding the compiler `float` (not `double`) lets it auto-vectorise for free — 90% of the win, 0% of the maintenance cost. |

**The one to defend:** *selective precision (INT8 here, FP32 there).* The naive move is "quantize everything for speed." The right move is **match precision to the job**: quantize the throughput-critical, error-tolerant stage (detection) and protect the precision-critical stage (depth). Blanket quantization would either slow you down or silently corrupt your geometry.
