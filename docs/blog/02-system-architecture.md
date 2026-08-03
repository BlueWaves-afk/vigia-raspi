In the previous episode, I talked about the motivation behind building autonomous intelligence on edge devices and why constraints like CPU-only inference, thermal limits, and unreliable connectivity fundamentally change how systems must be designed.

In this episode, we move from *why* to ***how***.

This article is about system architecture, how to design a real-time perception system that can run continuously on constrained hardware like a Raspberry Pi 4.

* * *

### The Big Picture

At a very high level, the system performs the following steps:

1.  There is a camera feed or video input, continuously streaming frames into the system.
    
2.  Frames are captured and passed through the lightweight, low latency models; In our system we use YOLO26n and MIDASv2.1 small.
    
3.  The models provide different signals that we use to perform Semantic, Geometric and Temporal Analysis of the streamed footage.
    
4.  The different signals are analysed and passed to a fusion engine that combines the scores to a rough prediction score. The system then alerts the user of its decision.
    

However, real systems are not simple linear pipelines. Different components run at different speeds(Midas and YOLO have varying latencies), and if everything is executed sequentially, the system becomes unusably slow. ( We have to wait for midas to execute, which causes lag or frame drops, causing us to miss vital pothole frames).

So instead of a linear pipeline, the system is designed as an asynchronous multi-threaded perception architecture.

![](https://cdn.hashnode.com/uploads/covers/697a3a7c4b19a21e89e9cf6f/f3533f5e-2101-4947-bf04-d8948f5d485e.svg align="center")

* * *

### The Core Design Principle

> The most important design decision in this system was that *Not every model needs to run on every frame.*

Object detection(YOLO) must run frequently. Depth estimation(MiDaS) is expensive and can run less frequently. Temporal reasoning happens across frames. Fusion combines signals over time.

Once you accept this idea, the architecture becomes much more efficient and practical.

* * *

### Thread Architecture

The system is built around multiple threads, each responsible for a specific part of the pipeline.

| Thread | Responsibility |
| --- | --- |
| Capture Thread | Reads frames from camera/video |
| Process Thread | Runs object detection and schedules depth tasks |
| Depth Thread | Runs MiDaS depth inference and geometry analysis |
| UI / Output Thread | Visualisation / logging |
| OpenVINO Worker Threads | Internal parallel inference execution |

The important part is that capture, detection, and depth estimation are not **blocking each other.**

This allows the system to maintain a stable frame rate even when some components are slow.

* * *

### **Frame Flow Through the System**

The pipeline works roughly like this:

The **stride check** in the Process Thread is the pipeline's key design decision. YOLO runs on every frame, but MiDaS only receives frames at every Nth index (e.g., every 3rd or 5th). This is because depth estimation is too expensive to run at camera frame rate, so the system decouples detection speed from depth quality.

The **Frame Buffer** between Capture and Process is a ring buffer — Capture writes continuously and Process always reads the latest frame, potentially dropping stale ones. This is intentional: you'd rather process the newest frame than queue up a backlog.

The **MiDaS Queue** is the second decoupling point. Process pushes work in and moves on immediately — it never waits for depth. The MiDaS Thread drains the queue at its own pace, which is why the temporal history update at the bottom is important: it accumulates state across the slower, strided depth frames to reconstruct motion over time.

This architecture allows the system to stay real-time even on CPU-only hardware.

![](https://cdn.hashnode.com/uploads/covers/697a3a7c4b19a21e89e9cf6f/4050582c-b6d5-454d-924b-7df64514f90c.svg align="center")

* * *

### Asynchronous Pipeline Design

If we ran everything sequentially like this:

*Capture → YOLO → MiDaS → Geometry → Temporal → Fusion → Next Frame*

The system would run at maybe 2–3 FPS, which is inefficient for real-time perception and dangerous for real world cases.

Instead, the system uses *frame buffers, work queues, separate threads, stride scheduling.*

So while YOLO is running on frame N, MiDaS might be processing frame N−3, and temporal analysis might be updating based on the last 10 frames.

This is how real autonomous systems operate, they are time pipelines, not frame pipelines.

![](https://cdn.hashnode.com/uploads/covers/697a3a7c4b19a21e89e9cf6f/ebbddbf3-e09f-453f-8635-24341a7e4636.svg align="center")

* * *

### Multi-Model Fusion

The system does not rely on a single model to decide whether something is a hazard.

Instead, it combines three signals:

| Signal | Source |
| --- | --- |
| Detection Confidence | Object Detection Model |
| Geometry Confidence | Depth + plane residual analysis |
| Temporal Confidence | Stability across multiple frames |

These are combined into a single score called the ***RRI (Risk Reliability Index)***.

Conceptually:

RRI = 0.40 × Detection Confidence

*   0.35 × Geometry Confidence
    
*   0.25 × Temporal Confidence
    

This is important because *detection alone* can produce false positives, *depth alone* can be noisy*, temporal stability* alone is not enough but together, they produce much more **reliable decisions.**

> This is a common pattern in autonomous systems, no single sensor or model is trusted. Decisions are made through fusion or an ensemble of models.

* * *

### Temporal Reasoning

One of the biggest problems in perception systems is frame-by-frame **instability**.

A pothole might be detected in one frame, missed in the next, and detected again in the third frame. If the system reacts instantly to every detection, it becomes ***unstable and unreliable.***

So instead, the system keeps a sliding window history of detections and geometry metrics.

From this history, it computes:

*   Persistence (how consistently the object appears)
    
*   Stability (how stable the geometry measurements are)
    

This allows the system to prefer stable hazards over noisy detections.

In other words:

> A slightly weaker detection that appears consistently over time is more trustworthy than a strong detection that appears only once.

This is how many real-world autonomous systems reason about the environment.

* * *

### Thermal-Aware Scheduling

One of the biggest problems when running AI on small edge devices is heat.

Edge devices crash because of:

*   Thermal throttling
    
*   Power limits
    
*   Memory pressure
    
*   Overloaded CPU
    

So the system includes a thermal-adaptive scheduling system.

The device temperature is periodically read from the system, and the depth inference frequency is adjusted dynamically.

For example:

| Temperature | Depth Inference Stride |
| --- | --- |
| Cool | Run depth often |
| Warm | Run depth less often |
| Hot | Run depth rarely |
| Critical | Reduce workload significantly |

Temperature Depth Inference Stride Cool Run depth often Warm Run depth less often Hot Run depth rarely Critical Reduce workload significantly This allows the system to adapt to hardware conditions in real time, rather than running at a fixed workload and eventually throttling or crashing.

This is a very important concept for edge AI systems.

> Your algorithm must adapt to the hardware, not the other way around.

* * *

### Frame Buffer and Queues

To support asynchronous execution, the system uses:

*   Frame buffers
    
*   Work queues
    
*   Ring buggers for instrumentation and visualization
    

The frame buffer ensures that:

*   The latest frame is always available
    
*   Slow components don’t block fast components
    
*   The system can handle temporary slowdowns
    

Queues are used to pass work between threads without blocking the pipeline.

This makes the system behave more like a stream processing system rather than a simple program.

* * *

### Why This Architecture Matters

The goal of this project was not to build the best pothole/road anomaly detector.

The goal was to design a system that:

*   Runs continuously
    
*   Handles multiple models
    
*   Works on CPU-only hardware
    
*   Adapts to thermal limits
    
*   Maintains real-time performance
    
*   Produces stable decisions over time
    
*   Can be extended to other applications
    

This same architecture could be used for drones, smart infrastructure monitoring, agriculture robots, industrial inspection, wildlife monitoring, dashcam analytics, autonomous robots, etc.

The real product is not the model. The real product is the *architecture* that makes autonomous intelligence possible on constrained hardware.

* * *

### Closing Thoughts

If there is one idea I want you to take from this article, it is this:

> Real-world AI systems are not model problems. They are systems engineering problems.

The model might be 5% of the code. The pipeline, scheduling, buffering, threading, thermal management, logging, and testing make up the rest.

In the next episode, I will dive into CPU-only inference and optimization on ARM processors, and what it actually takes to squeeze real-time performance out of edge hardware without a GPU.

---

## 🎓 CS Fundamentals — study companion

*This episode is a goldmine for **Operating Systems** and **Data Structures**, with a side of **Computer Architecture** and **System Design**. If you understand every term below, you can hold your own on concurrency, scheduling, and pipeline design in any interview.*

### Operating Systems (OS)

**What this post touches:** threads, concurrency, the producer–consumer problem, scheduling, CPU affinity, thermal/DVFS-style governors.

**Deep dive.**
- **Process vs thread.** A *process* has its own address space; *threads* within a process share memory but have their own stack and registers. This system uses **threads** (capture, process, depth, output) precisely because they share the frame buffers cheaply — no IPC needed. The cost of that sharing is that you must synchronise access (Episode 4).
- **The producer–consumer problem.** The capture thread *produces* frames; the process/depth threads *consume* them. This is the canonical OS synchronization problem, classically solved with a bounded buffer + semaphores/condition variables. The blog's ring buffer and work queues are concrete instances of it.
- **Concurrency vs parallelism.** *Concurrency* = multiple tasks in progress (interleaved); *parallelism* = literally running at the same instant on multiple cores. The pipeline is both: concurrent by design, parallel because threads are pinned to different cores.
- **CPU affinity / core pinning.** Binding a thread to a specific core keeps its data warm in that core's cache and avoids the scheduler migrating it (which would cause cache-cold stalls). It also isolates the heavy inference thread from the UI thread.
- **Scheduling.** The OS scheduler decides which thread runs when. A general-purpose scheduler (CFS on Linux) is fair but not deterministic; real-time work wants a priority scheduler (Episode 6). The blog's **stride scheduling** ("run MiDaS every Nth frame") is application-level scheduling layered on top.
- **Adaptive control / feedback loops.** The **thermal-aware scheduler** reads temperature and adjusts the depth stride — a closed feedback loop, the same idea as OS **DVFS** (Dynamic Voltage & Frequency Scaling) governors that trade performance for thermal headroom. Key phrase: *the algorithm adapts to the hardware, not the other way around.*

**Interview Q&A.**
1. *Process vs thread — when do you pick threads?* → When tasks must share memory cheaply and you want low context-switch cost; here, sharing frames between stages.
2. *What is the producer–consumer problem and how is it solved?* → Coordinating a producer and consumer through a bounded buffer without races or lost/duplicated items; solved with a mutex + condition variables / semaphores (empty/full counts).
3. *Concurrency vs parallelism?* → Interleaving vs simultaneous execution.
4. *Why pin a thread to a core?* → Cache warmth + avoid migration cost + isolation; downside is less scheduler flexibility.
5. *What is a feedback/adaptive scheduler?* → It measures a runtime signal (temperature, load) and adjusts workload; DVFS is the OS analogue.

### Data Structures & Algorithms (DSA)

**What this post touches:** ring (circular) buffers, work queues, the sliding-window pattern, "latest-wins" vs FIFO.

**Deep dive.**
- **Ring / circular buffer.** A fixed-size array with a modulo index (`i % N`). O(1) push/read, no allocation after setup, cache-friendly (contiguous). The **capture→process** hand-off uses it with *latest-wins* semantics — the writer overwrites old frames and the reader always takes the newest. This is the right choice when *stale data is worthless*.
- **Queue (FIFO).** The **process→depth** hand-off uses a queue: work is processed in order, decoupling a fast producer from a slow consumer. Contrast with the ring: a queue *preserves* items (you want every depth request handled), a latest-wins ring *drops* them (you only want the freshest frame).
- **The design lesson:** *choosing the data structure encodes your latency policy.* Latest-wins ring → minimise lag, tolerate drops. FIFO queue → guarantee processing, tolerate backlog. Picking wrong is how you get lag.
- **Sliding window.** Temporal reasoning keeps the last *k* frames — a sliding window — and computes aggregate stats over it. O(k) or O(1) with incremental updates. (Explored fully in Episode 5.)

**Interview Q&A.**
1. *Implement a fixed-size ring buffer.* → Array + head/tail (or index + count) + modulo; discuss full/empty disambiguation.
2. *Ring buffer vs queue — when each?* → Latest-wins/drop-stale vs FIFO/preserve-all; map to "freshness matters" vs "every item matters."
3. *How do you decouple a fast producer from a slow consumer?* → Bounded queue + backpressure (drop, block, or grow).

### System Design

- **Pipeline / stream processing.** The system is a **dataflow pipeline** of stages connected by buffers — the same architecture as a CPU instruction pipeline or a stream processor (Kafka/Flink at scale). Stages run asynchronously so a slow stage doesn't stall a fast one.
- **Multi-model fusion / ensembles.** No single signal is trusted; detection + geometry + temporal are fused into one score (RRI). This is **sensor fusion / ensembling** — combining weak, independent signals into a stronger decision, which also improves robustness to any one signal failing.
- **Backpressure & load shedding.** When overloaded, the system *sheds load* (increases stride, drops stale frames) instead of collapsing — a core resilience pattern (graceful degradation).

**Interview Q&A.**
1. *Design a streaming pipeline where stages have different speeds.* → Decouple with queues/buffers, run stages on separate workers, add backpressure/load-shedding, size buffers to the slowest stage's latency window.
2. *Why ensemble multiple models/signals?* → Lower variance, fewer correlated failures, graceful degradation.

### Computer Architecture (COA) — cameo
- **Instruction pipeline analogy.** While YOLO runs on frame N, MiDaS runs on N−3 and temporal analysis on the last 10 — this is literally **pipelining** (overlapping stages of different instructions), the same idea CPUs use to hit high throughput despite multi-cycle instructions.

### Quick-review flashcards
- **Producer–consumer** → bounded buffer + condition variables.
- **Concurrency ≠ parallelism.**
- **CPU affinity** → cache warmth + no migration.
- **Ring (latest-wins)** vs **queue (FIFO)** → freshness vs completeness.
- **Sliding window** → last *k* items, aggregate.
- **DVFS / adaptive scheduling** → measure a signal, adjust workload.
- **Backpressure / load shedding** → degrade instead of collapse.

### ⚖️ This vs That — the architecture decisions, and the roads not taken

| Decision | Alternatives | Why this choice |
|---|---|---|
| **Async multi-threaded pipeline** | One sequential loop (capture→YOLO→MiDaS→fuse) | Sequential is trivial to write and caps you at ~2–3 FPS because every stage waits for the slowest. Async lets a slow stage run behind a fast one — the only way to stay real-time. |
| **Threads across cores** | Single-threaded async I/O event loop (à la Node.js) | An event loop wins for **I/O-bound** work, but inference is **CPU-bound** — you need genuine parallelism on multiple cores, which an event loop on one core can't give you. |
| **Stride scheduling (MiDaS every Nth frame)** | Run every model every frame; drop frames uniformly | Depth at 525ms can't run at camera rate. Uniform frame-dropping throws away detections too. Striding *only the expensive stage* keeps detection fast and depth "good enough." |
| **Multi-signal fusion (RRI)** | One end-to-end model that outputs "hazard: yes/no" | A single model is simpler but a black box — brittle to false positives and impossible to degrade gracefully when a sensor drops. Fusing independent signals is robust and explainable. |
| **Latest-wins ring + work queue** | One shared queue for everything | Different hand-offs want different semantics: freshness (drop-stale ring) for capture, completeness (FIFO queue) for depth work. One structure can't be both. |

**The one to defend:** *async vs sequential.* Interviewers love "your pipeline is slow — speed it up." The junior answer optimises the model. The senior answer: **the model wasn't the bottleneck; the sequential structure was.** Decouple the stages with buffers so the 525ms stage runs in the background and the fast stage sets the frame rate. That reframing — from "faster model" to "better dataflow" — is the entire point of Episode 2.
