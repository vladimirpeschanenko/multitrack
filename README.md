# multitrack

Low-latency, lock-free, multi-object tracker that ingests `(t, x, y)` detections, fits an online constant-velocity model per object, and predicts rectangle entry.
Design goals: **no virtuals**, **O(1)** updates, **SPSC ring**, **per-object worker threads**, and **cache-line safety**.

# Directory structure

```
multitrack
 ├─ cli       ── command line interface
 ├─ core      ── math/logic that’s independent of I/O and threading
 ├─ docs      ── requirements
 ├─ engine    ── runtime orchestration and concurrency
 ├─ geom      ── iny geometry value types
 ├─ input     ── provided input files (rec_v2025.txt, track_v2025.txt)
 ├─ io        ── input sources (no vtables; same next(TrackPoint&) -> int contract)
 ├─ output    ── copy of console output for provided input
 ├─ sinks     ── output policies (templated, no virtuals)
 └─ util      ── small, reusable helpers
```

## Build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run
```bash
./build/multitrack file:<path> <rect.txt>
```
