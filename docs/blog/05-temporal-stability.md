# Riding the Blue Wave: Building Autonomous Intelligence on the Edge #05

*Episode 5: Temporal Stability vs Raw Confidence — why a steady, unremarkable detection beats a brilliant one that flashes for a single frame.*

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
