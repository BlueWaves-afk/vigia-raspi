## Welcome.

*Join me as I surf the digital wave; designing systems, breaking code, and experimenting with modern software to see what truly works in the real world.*

With the rise of modern Artificial Intelligence and the open source nature of many of the ML models; The common man has great opportunities to build true solutions that solve real world issues. In this blog series we explore the capability and push the limits of **edge constrained systems**. How do you build a sophisticated, real-time autonomous perception system that can flag objects and detect anomalies using ***cheap, power-constrained ARM CPUs?***

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770401927591/981a1836-7549-4201-bb95-322d66956209.gif align="left")

> our end-to-end system visuals.

<details data-node-type="hn-details-summary">
<summary>Dependencies</summary>
<p>For the software system I use native <code>C++</code> implementation of open source models like <code>yolo26</code> and <code>MiDaS</code> with several optimisations, for hardware side i use a <code>Raspberry Pi 4</code>, camera modulev2, Cooling unit, micro SD card. More details in upcoming episodes, but if you wish to emulate this, it is necessary you have these dependencies.</p>
</details>

For our purpose, we will explore the problem of Road Hazard Detection and flagging; An apt and real world problem that plagues my country. We try to build this system on hardware as modest as the `Raspberry Pi 4`.

* * *

## Why This Problem Matters?

Most modern computer vision projects assume one of the below;

1.  A powerful ***GPU***.
    
2.  A reliable cloud ***backend***(*Azure, AWS, GCP, etc*).
    

However, real world systems, atleast economically constrained ones, do not always have these.

*   Dashcams
    
*   Agricultural Drones
    
*   Smart Infrastructure
    
*   Edge analytics for developing regions
    

These systems have to perform under hard constraints, like

*   Limited Compute(*cheap CPUs*).
    
*   Tight Thermal budgets(*inexpensive cooling units*).
    
*   Strict Latency requirements(*eg; fast moving environments*).
    
*   Unreliable connectivity.
    

This project explores what it *actually* takes to implement autonomous intelligence on these **edge-constrained environments.**

* * *

## Why Road Hazard Detection?

The choice of this problem statement was two-fold.

Firstly, I wanted to expand my work in a previous project that related to road hazard detection, which won me first place in the `imobilothon 5.0` hackathon that took place last October to December.

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770394950770/6f8fd7ff-f868-4020-9ced-b4050a7af7b5.png align="center")

> PUNE at Embassy Techzone **Hinjawadi** DEC, 2025

Secondly the problem of road hazard/obstacle detection was a problem that is also tackled by complex ADAS systems, hence it proved to be a technically complex, multi-modal and time-sensitive problem.

However, the principles I used to design the architecture isn’t exclusive to road hazard detection, the same system can be broadly expanded into other physical applications that can have great impact in the real world.

Swap ‘Potholes’ for;

*   Manufacturing defects.
    
*   Debris.
    
*   Infrastructure Damage.
    
*   Wildlife.
    

The broad system remains the same, which is why the real focus is the **system**.

## The Real Focus: System Design Over Models

This project puts less emphasis on any singular ML model; but rather it focuses on the principles of designing and end-to-end feasible system. It focuses on aspects like;

*   Multimodal Pipeline Architecture.
    
*   Temporal Reasoning(Tracking objects over time).
    
*   Asynchronous Execution(different modals output at different rates).
    
*   Testability.
    
*   Observability.
    

> *"Only a small fraction of real-world ML systems is composed of the ML code... The required surrounding infrastructure is vast and complex".*

**It is my great belief that a slightly worse model in a well designed system will outperform a great model inside a poorly structured pipeline.**

![](https://cdn.hashnode.com/res/hashnode/image/upload/v1770407771066/b9fdbace-4577-4cb3-a3ff-d22993219114.jpeg align="center")

## Constraints as a Feature, Not a Bug

Rather than fighting hardware limits, I embrace and plan for edge cases.

The system was designed with these assumptions in mind;

*   CPU-only inference.
    
*   Strict FPS caps.
    
*   Thermal throttling.
    
*   Bounded Memory.
    
*   Latency.
    

These issues, force me to write better code, and design a better mental modal of the overall architecture; they force ***clarity.***

**Many optimisations you would never bother with on a GPU or higher compute suddenly becomes essential.**

* * *

## What this Blog Series Will Cover?

This is an episodic series, as I wish to cover each decision and concept in depth while talking about the architecture. The upcoming episodes will go more in depth and will be linked here, they will cover topics like;

*   Designing for CPU-only Inference.
    
*   Asynchronous pipelines without chaos.
    
*   Temporal stability vs raw confidence.
    
*   Testing autonomous systems end-to-end.
    
*   Visual Debugging for perception pipelines.
    
*   Performance profiling on ARM.
    
*   Why most “real-time” demos aren’t actually real-time.
    

Everything will be covered and grounded in real code, real benchmarks, and real failures.

## Who This Is For?

This series is for:

*   `Engineers building edge AI systems.`
    
*   `Robotics Developers.`
    
*   `Embedded system engineers.`
    
*   `Anyone tired of high cost hardware implementations.`
    

If you care about real, autonomous systems, capable of handling edge cases you are in the right place ❤️.

* * *

Road hazard detection is just the story I am telling.

The real journey and learning is on how to build autonomous intelligence that survives constraints, and optimises resources.

In the next episode, I will dive into the complete architecture and design choices of the system.

[our github repo](https://github.com/BlueWaves-afk/vigia-raspi.git)

---

## 🧰 The stack, from zero — and what I chose it over

Before the architecture episodes go deep, here's the whole toolbox in one place, and the road not taken for each. This is the "what frameworks did you use, and why not the obvious ones" tour.

- **Language — C++17 (over Python/PyTorch).** Python is faster to prototype, but its GIL serialises threads and the interpreter injects unpredictable latency — both fatal for a real-time loop. C++ buys deterministic latency, true multi-core threads, and manual memory control. Built with **CMake**, cross-compiled for the Pi 5's Cortex-A76 with `-mcpu=cortex-a76 -O3 -ftree-vectorize` so the compiler auto-vectorises to **NEON** SIMD.
- **Inference runtime — ONNX Runtime + ARM Compute Library / KleidiAI (over OpenVINO and TFLite).** The hackathon baseline used **OpenVINO** with a forced FP32 hint — not ideal on an A76 and awkward to quantise. We standardised on **ONNX Runtime** (C++) with the ACL/KleidiAI execution provider for **INT8 UDOT** dot-product acceleration, and zero-copy IO-binding on the hot loop. Honest status: this migration is in progress — the current build still links OpenVINO while the ONNX/KleidiAI path lands. **TFLite** was the other option; ONNX won on C++ ergonomics and the ARM INT8 story.
- **Models — YOLO (INT8) for detection, MiDaS (FP32) for monocular depth.** Two models, two precisions, fused later (Episode 2). Open-source weights, native inference, no vendor lock.
- **Vision — OpenCV** for capture, colour-space, and ROI work.
- **Runtime & concurrency — ROS2 nodes + a PREEMPT_RT kernel.** Each node runs on a dedicated `std::thread` with `SCHED_FIFO` priority and CPU affinity, on a real-time kernel (Episode 4). The priority ladder is explicit: anti-death > sensor-bridge > camera > vision/depth > fusion.
- **Sync & integrity — libcurl (HTTPS event uplink) + OpenSSL (HMAC event signing).** Only *verified events* leave the device — a few signed bytes, never the video.

## 🚢 From demo to production

The gap between "runs on my Pi at the demo table" and "survives a month bolted to a windshield" is its own discipline, worth naming from zero:
- **Reproducible cross-compilation** — a pinned toolchain and CMake cache so the same binary builds anywhere, not just on the one Pi it was hacked on.
- **CPU feature-gating** — the INT8 fast path is guarded behind a `/proc/cpuinfo asimddp` CPUID check, so the binary degrades gracefully on a chip without dot-product, instead of crashing.
- **Deployment as a service** — shipped as a `systemd` unit (or a container) that restarts on failure and starts on boot, not a terminal command someone runs by hand.
- **Thermal & power management** — the scheduler has to plan for throttling, because on the edge the chip *will* get hot (Episode 3/6).
- **Observability & OTA** — structured logs, health metrics, and a safe over-the-air update path for a device you can't plug a keyboard into.

Honesty is part of the bar: the ONNX/KleidiAI migration is mid-flight, and a few subsystems still carry hardware-access-pending flags. Naming that precisely — rather than claiming the demo is finished — is the same engineering maturity the rest of this series argues for.

---

## 🎓 CS Fundamentals — study companion

*The article above is the story of **why** edge AI is hard. This section turns that story into interview-ready theory. Revisit it before placement season and you should be able to answer core questions on **Computer Architecture (COA)**, **Computer Networks (CN)**, **Operating Systems (OS)**, and **System Design** drawn from this one motivating problem.*

### Computer Organization & Architecture (COA)

**What this post touches:** CPU vs GPU, why ML wants a GPU, power/thermal budgets, ARM vs x86, the memory wall.

**Deep dive.**
- **CPU vs GPU — latency vs throughput machines.** A CPU has a few "fat" cores optimised for *latency*: deep pipelines, large caches, branch prediction, out-of-order execution — it makes a single thread finish fast. A GPU has thousands of "thin" cores optimised for *throughput*: it runs the *same* instruction across huge batches of data (SIMT — Single Instruction, Multiple Threads). Neural-network inference is mostly matrix multiplication — thousands of identical multiply-accumulates — which is exactly the embarrassingly-parallel workload a GPU eats for breakfast and a CPU grinds through serially. That's *why* "just add a GPU" is the default, and why not having one changes everything.
- **The roofline / memory wall.** Performance is bounded by either compute (FLOPs/s) or memory bandwidth (bytes/s). Big models on weak hardware are usually **memory-bound**: the cores stall waiting for weights to arrive from RAM. This is why on a Pi the *data movement* dominates, not the arithmetic (a theme that returns in Episode 3).
- **Power & thermals as a first-class constraint.** Dynamic power ≈ `C·V²·f`. Push frequency `f` up and power (and heat) rise super-linearly. A phone/Pi has no room to dump that heat, so the chip **throttles** `f` to stay under a thermal limit — trading performance for survival. On a datacentre GPU this is someone else's problem; on the edge it's *your* scheduler's problem (Episode 2).
- **ARM vs x86.** ARM is a **RISC** ISA (load/store, fixed-width, many registers, power-efficient) that dominates mobile/edge; x86 is **CISC** (variable-length, rich instructions) that dominates desktop/server. The Pi's Cortex-A72/A76 are ARM — which is why the whole series obsesses over ARM-specific tricks (NEON, INT8 dot-product).

**Interview questions you can now answer.**
1. *Why are GPUs faster than CPUs for deep learning?* → NN inference is massively data-parallel matrix math; GPUs expose thousands of SIMT lanes with high memory bandwidth, whereas a CPU has few latency-optimised cores. CPUs win on branchy, serial, latency-sensitive code.
2. *What does it mean for a workload to be "memory-bound" vs "compute-bound"?* → Bound by bandwidth vs by FLOPs; use the roofline model; edge inference is typically memory-bound.
3. *Why does a chip run slower when it gets hot?* → Thermal throttling reduces clock frequency to keep power `∝ V²f` within the thermal design limit.
4. *RISC vs CISC — one practical consequence?* → Simpler fixed-width RISC instructions decode cheaply and save power (great for edge); CISC packs more work per instruction.

### Computer Networks (CN)

**What this post touches:** why "unreliable connectivity" pushes intelligence to the edge; the cloud round-trip.

**Deep dive.**
- **The latency argument for edge.** A cloud inference round-trip is: capture → serialize → TCP/TLS handshake → upload over a possibly-2G link → queue → infer → download → act. Each hop adds latency and a failure point. For a *fast-moving, safety-critical* task, the round-trip time can exceed the time-to-impact. **Edge computing** removes the network from the critical path — you infer locally and only sync results opportunistically.
- **Bandwidth & cost.** Streaming raw video to the cloud is enormous bandwidth; sending only *verified events* (a few bytes) is tiny. This is the "compute where the data is" principle — process at the edge, transmit conclusions.
- **The CAP-style tradeoff at the edge.** With unreliable connectivity you must choose to keep working (availability) during a partition rather than block on the cloud (consistency). Edge systems favour **availability + local autonomy**, syncing when the link returns (eventual consistency).

**Interview Q&A.**
1. *When would you do inference on-device instead of in the cloud?* → Tight latency budget, unreliable/expensive connectivity, privacy, high raw-data volume. Cloud when you need big models, central aggregation, or the device is too weak.
2. *Why is edge better for privacy/bandwidth?* → Raw data never leaves the device; only derived results are transmitted.

### System Design

- **"ML code is ~5% of a real ML system."** (Sculley et al., *Hidden Technical Debt in ML Systems*.) The model is surrounded by data pipelines, serving infra, monitoring, and glue. The blog's thesis — *a slightly worse model in a well-designed system beats a great model in a bad one* — is a system-design principle: optimise the **pipeline**, not just the model.
- **Constraints as a design forcing-function.** Hard limits (CPU-only, thermal, memory, latency) remove ambiguity and force clear architecture. Good interview framing: *"I treat the tightest constraint as the design axis."*

**Interview Q&A.**
1. *Design a real-time perception system for a $50 device.* → Lead with constraints (CPU-only, thermal, latency budget), pick a cheap model, then spend your design effort on the pipeline: async stages, buffering, graceful degradation, observability.

### Quick-review flashcards
- **SIMT** = Single Instruction, Multiple Threads (GPU execution model).
- **Roofline** = performance bounded by min(compute, bandwidth).
- **Dynamic power** ∝ `C·V²·f` → why throttling works.
- **Edge computing** = compute near the data source to cut latency/bandwidth and survive partitions.
- **RISC (ARM)** vs **CISC (x86)**.
- **Sculley's law (informal):** the model is ~5% of an ML system.

### ⚖️ This vs That — the architecture decisions, and the roads not taken

*The heart of every "why did you build it this way" interview. For each big fork, here's what else was on the table and why I went the way I did.*

| Decision | Alternatives | Why this choice |
|---|---|---|
| **Edge inference** | Cloud inference; hybrid | Cloud gives you unlimited model size but puts the *network on the critical path* — fatal for a safety task on unreliable rural links. Edge = bounded latency, works offline, private, cheap to run. |
| **CPU-only** | GPU / Google Coral TPU / an NPU stick | Accelerators are faster but cost money, draw power, and aren't universally available — which *breaks the whole "affordable, accessible" thesis*. Proving it on a bare CPU is the harder, more defensible claim. |
| **Native C++** | Python + PyTorch | Python is faster to prototype but the GIL blocks true multi-core parallelism and the interpreter adds unpredictable latency. C++ buys deterministic latency, real threads, and manual memory control — the things a real-time edge system lives or dies on. |

**The one to be able to defend out loud:** *edge vs cloud.* The instinct in interviews is "just use the cloud, it's more powerful." The senior answer is: **put the network on the critical path only if you can afford to lose it.** For a hazard 20 metres ahead on a road with no signal, you can't — so intelligence moves to the edge, and the cloud becomes an optional, eventually-consistent sync layer, not a dependency. That single tradeoff (latency + availability + privacy vs raw model power) is the whole reason this series exists.
