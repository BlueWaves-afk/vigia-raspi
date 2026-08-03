Every object detector gives you a confidence score, and the beginner's instinct is to trust it directly: high score means hazard, act on it. On a single image, fine. On a live video stream from a bouncing camera on an Indian road, that instinct will drive you insane — and possibly off the road. This episode is about the dimension a raw confidence score ignores: time.

## The problem is flicker

Point a detector at a real pothole through a moving camera and watch what happens frame by frame. Detected at 0.82. Missed entirely on the next frame — motion blur, a bad angle, a shadow. Back at 0.71. Gone again. This flicker is not the model being bad; it is what single-frame perception looks like in the real world.

If the system reacts to every frame independently, it becomes a strobe light: alert, no alert, alert, no alert. That's useless to a driver and dangerous as a control signal. The detector is answering the wrong question. It's telling me "is there a pothole in *this frame*," when what I actually need to know is "is there a pothole *on this road*."

## Confidence is a point; intelligence is a trend

So the system doesn't trust any single frame. It keeps a short sliding-window history — the last ten frames or so — of detections and their geometry measurements (the depression depth and roughness from the depth analysis). From that window it computes two things:

- **Persistence** — how *consistently* the object shows up across the window. A hazard that appears in eight of the last ten frames is real; one that appeared once is probably noise.
- **Stability** — how *steady* the geometry measurements are. A genuine pothole's depth reading is consistent frame to frame; a false positive's numbers jump around.

The principle I kept coming back to is this: **a slightly weaker detection that appears consistently over time is more trustworthy than a strong detection that appears only once.** A steady 0.6 that holds for ten frames is a hazard. A spike to 0.9 for one frame and nothing after is a shadow. Raw confidence can't tell those apart. Time can.

## Feeding it into the fusion score

That temporal confidence isn't the whole decision — it's one voice in the fusion I introduced earlier. Since Episode 2 the fusion has actually grown a fourth term: it now blends YOLO's detection confidence, the depth-plane geometry confidence, this temporal confidence, and an impact-severity term derived from the gravity-compensated IMU — the physical jolt the vehicle actually felt. They're combined into the Road Risk Index (RRI) as a weighted sum.

The detail I'm proud of there is **graceful degradation.** Not every device has every sensor wired up — during bringup the IMU might be in stub mode, or there's no depth on a vision-only build. So each term carries a "have this?" flag, and the RRI renormalises its weights over only the terms that are actually present. A missing IMU doesn't zero the score; it just shifts the system to a vision-and-depth tier, and the output says which tier it's running in. The fusion never silently collapses because one input went dark — it tells you honestly how much of the picture it currently has.

## The unglamorous implementation note

Temporal reasoning sounds heavy — "history," "windows," "trends" — but it is astonishingly cheap. It is a ring of ten floats. The one mistake I made was reaching for a `std::deque` to hold the history, which spreads its elements across segmented memory and causes a cache miss on every segment boundary when you walk it. On a Cortex-A72, that's exactly the kind of small sloppiness the hardware punishes. Swapping it for a fixed-size circular `std::array` — contiguous memory, no heap, ten floats sitting next to each other — made the persistence and stability calculations essentially free. Cheap enough that there's no excuse *not* to reason over time.

## Closing Thoughts

The biggest reliability win in the whole system did not come from a better model. It came from refusing to trust any single frame and asking, instead, whether a hazard holds up over time. Persistence and stability turn a flickering detector into a steady decision, and folding them into a self-renormalising fusion score means the system stays honest even when a sensor drops out. Confidence tells you what one frame thinks. Temporal reasoning tells you what the road is actually doing.

In the next episode — the one I think every edge-AI builder should read — I'll make an uncomfortable confession about benchmarks, and dig into why most "real-time" demos, including my own first attempt, aren't actually real-time.

*our [github repo](https://github.com/BlueWaves-afk/vigia-raspi).*

---

## 🎓 CS Fundamentals — study companion

*This episode is mostly **Data Structures & Algorithms** (the sliding-window pattern, circular buffers, cache locality) with a dip into **signal processing / ML** (temporal filtering, ensembles) and **System Design** (graceful degradation).*

### Data Structures & Algorithms (DSA)

**What this post touches:** sliding-window pattern, circular buffer vs `std::deque`, cache locality, incremental aggregates, weighted moving average.

**Deep dive.**
- **The sliding-window pattern.** Keep the last *k* items and compute an aggregate (count, mean, variance) over them; as a new item arrives, the window slides. It's one of the most common interview patterns (max-in-window, moving average, longest-substring). Here the window holds the last ~10 frames of detections and geometry.
- **Persistence & stability as window stats.** *Persistence* = how many of the last *k* frames contained the detection (a **count/mean** over a boolean window). *Stability* = how little the geometry varies (a **variance / standard deviation** over the window). Both are O(k) to recompute, or O(1) if you maintain running sums incrementally (add the incoming, subtract the outgoing).
- **Circular buffer vs `std::deque` — the cache-locality lesson.** A `std::deque` stores elements in **segmented** chunks scattered in memory; walking it for the window stats causes a **cache miss** at each segment boundary. A fixed `std::array<float, 10>` + head index is **contiguous** — the whole window fits in a cache line or two, and iteration is prefetch-friendly. Same Big-O, very different real speed on a Cortex-A72. *Asymptotic complexity ≠ constant factors; on real hardware the constant is memory locality.*
- **Why not just raise the confidence threshold?** Because a threshold is memoryless — it can't tell a steady 0.6 (real) from a one-frame 0.9 spike (noise). The window adds the **time dimension** a point-estimate lacks.

**Interview Q&A.**
1. *Describe the sliding-window technique and a problem it solves.* → Maintain an aggregate over the last *k* elements with O(1) updates; e.g., moving average, max in window (monotonic deque), longest-substring-without-repeat.
2. *Compute a moving average in O(1) per step.* → Keep a running sum; on new element add it and subtract the one leaving the window.
3. *Two structures with the same Big-O can differ 10× in speed — why?* → Constant factors: cache locality, allocation, branch prediction. `deque` (segmented) vs `array` (contiguous) is the textbook case.
4. *When is a circular buffer the right choice?* → Fixed-capacity, FIFO-ish, O(1) push/pop, no allocation, cache-friendly — bounded histories and streams.

### Signal Processing / ML systems (bonus, common in ML-role interviews)
- **Temporal filtering.** Requiring persistence is a **low-pass filter** over detections — it suppresses high-frequency flicker (noise) and passes stable signal. Same family as a moving average or EMA.
- **Ensembling / late fusion.** Combining detection + geometry + temporal (+ IMU) into the RRI is a weighted **ensemble**; independent weak signals reduce variance and correlated failure. "A weak-but-consistent signal beats a strong-but-lone one" is the bias–variance intuition.
- **The precision/recall tradeoff.** Temporal gating trades a little **recall** (you might delay flagging a real one-frame hazard) for a lot of **precision** (far fewer false alarms) — the right trade when false alarms erode trust.

**Interview Q&A.**
1. *How do you stabilise a noisy per-frame classifier?* → Temporal smoothing: majority vote / persistence over a window, or EMA of the score; trades recall for precision.
2. *What is late fusion / ensembling and why does it help?* → Combine independent models' outputs; lower variance, robustness to any single failure.

### System Design
- **Graceful degradation via renormalisation.** The RRI carries a "have this input?" flag per term and renormalises weights over present terms, plus a **tier** (`vision-only / vision+depth / full`) reported downstream. So a dead sensor **degrades** the score's confidence rather than **zeroing** it. This is a core resilience pattern: *partial input → partial (labelled) answer, never a crash or a silent wrong answer.*

**Interview Q&A.**
1. *Design a scoring system that still works when some inputs are missing.* → Per-input presence flags, renormalise weights over available inputs, expose a degradation tier so consumers know the confidence level.

### Quick-review flashcards
- **Sliding window:** last *k* items, O(1) incremental aggregate.
- **Persistence = mean over boolean window; stability = variance over window.**
- **`deque` (segmented, cache-miss) vs `array` (contiguous, cache-hot)** — same Big-O, different constants.
- **Temporal gating = low-pass filter; trades recall for precision.**
- **Late fusion / ensemble** → lower variance, robust to single-signal failure.
- **Graceful degradation:** renormalise over present inputs, report the tier.

### ⚖️ This vs That — the architecture decisions, and the roads not taken

| Decision | Alternatives | Why this choice |
|---|---|---|
| **Temporal smoothing (persistence/stability)** | Raw per-frame confidence threshold | A threshold is memoryless — it can't distinguish a steady 0.6 from a one-frame 0.9 spike. A window adds the time dimension that turns flicker into a stable decision. |
| **Simple sliding-window stats** | Kalman filter; Bayesian tracker; SORT/DeepSORT | Kalman/SORT are more powerful trackers but carry state, tuning, and CPU cost. On a tight budget, cheap persistence + variance recover most of the stability for a fraction of the complexity. |
| **Circular `std::array`** | `std::deque`; linked list | Same Big-O, but the array is contiguous and cache-hot; the deque is segmented (cache misses) and the list is a pointer-chase. On an A72, locality is the real cost. |
| **Weighted-sum fusion (renormalised)** | Learned fusion (small NN); max; majority vote | A weighted sum is interpretable, allocation-free, and *degrades gracefully* by renormalising over present inputs. A learned fusion needs training data and turns the decision into a black box. |

**The one to defend:** *simple filter vs "proper" tracker (Kalman/SORT).* It's tempting to name-drop Kalman filters. The mature answer is **YAGNI on a constrained device**: a Kalman filter models motion you may not need, adds state and tuning, and costs cycles you don't have — while a sliding window of persistence + variance buys 90% of the robustness for a ring of ten floats. Choosing the *cheapest technique that solves the actual problem* is the signal.
