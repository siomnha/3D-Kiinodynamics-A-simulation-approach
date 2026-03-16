# Snapshot strategy and replan latency/resource analysis

This note answers the practical onboard question:

- after each snapshot period, should we overwrite one `.bt` or generate another `.bt`?
- does map reload introduce latency while flying?
- how to reduce storage and computational usage onboard?

---

## 1) Overwrite one `.bt` vs create new `.bt` files

## Option A — Overwrite one file (recommended default onboard)

Behavior:
- always save to the same file (e.g. `map_snapshot.bt`).

Pros:
- minimal storage growth,
- very simple file management,
- deterministic planner input path.

Cons:
- no history for debugging,
- if a write is interrupted, snapshot can be corrupted unless atomic write/rename is used.

Use when:
- onboard CPU/storage is constrained,
- map mostly static,
- operation priority is runtime simplicity.

## Option B — Rolling snapshots (`map_snapshot_00001.bt`, ...)

Behavior:
- write a new file each cycle, keep last N files.

Pros:
- allows rollback/debug,
- safer against partial write corruption,
- useful for post-flight analysis.

Cons:
- more filesystem I/O,
- more storage,
- need retention policy.

Use when:
- you still tune the stack and need traceability.

## Recommendation for your mission profile

For **static map + low-probability dynamic obstacles**, use:
- `mode=overwrite` in normal flight,
- `mode=rolling` only during tuning/flight-test campaigns.

---

## 2) Does map reload create latency during flight?

Yes, potentially. Latency points:

1. snapshot generation time (I/O + serialization),
2. planner map reload time,
3. planner recomputation time,
4. trajectory swap/stitch time.

If done synchronously in control-critical loops, it can cause visible lag.

### Latency mitigation checklist

1. **Rate-limit reloads** (`map_reload_min_interval_s`).
2. **Save-on-change-only** (skip identical maps).
3. **Background snapshot task** (non-blocking from trajectory publisher).
4. **Asynchronous planner reload** with status topic.
5. **Trajectory hold-last-safe segment** while new plan is prepared.
6. **Stitch new trajectory** instead of hard reset to avoid abrupt motion.

---

## 3) Storage and compute optimization policy (onboard)

## Minimum-resource profile

- snapshot mode: `overwrite`
- snapshot period: `5~10 s`
- save-on-change-only: `true`
- octomap resolution: keep as coarse as mission allows
- replan trigger: event-based (not every map frame)

## Medium-resource profile

- snapshot mode: `rolling`
- max snapshots: `3~5`
- snapshot period: `3~5 s`
- trigger still rate-limited

## Additional compute savers

- monitor occupancy only in a local corridor near active trajectory,
- trigger replan only for safety-relevant change classes,
- suppress repeated same-reason triggers within cooldown window.

---

## 4) Suggested operational policy now

1. Use global planner on snapshot map (`overwrite` mode).
2. Keep online `/octomap_full` for **monitoring/triggers** only.
3. On trigger:
   - request snapshot update (if changed),
   - reload planner map asynchronously,
   - compute replan and stitch.
4. If replan unavailable within timeout:
   - execute failsafe policy (hover/loiter/RTL).

This gives low-storage/low-compute operation while still preparing for dynamic-risk handling.
