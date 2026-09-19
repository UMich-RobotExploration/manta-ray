# TODO / Known Issues

Living document of bugs, semantic inconsistencies, and refactors we know about
but haven't fixed yet. Add new entries at the top of the relevant section.
Each entry should describe the issue, its impact, and (optionally) a proposed
fix so future readers can pick it up without re-deriving the diagnosis.

Sections are by package. `[status]` prefix: `open`, `wip`, `deferred`, `done`.
When closing an entry, move it under `## Resolved` at the bottom with a date
and a commit reference.

---

## mantaray (C++)

### [open] Sim clock uses `double` seconds; loses precision on non-binary-exact dt over long runs

**Location**: `mantaray/src/rb/RbWorld.cpp` (`simData.time`, `stepWorld`,
`advanceWorld`) and `mantaray/src/rb/helpers.{h,cpp}` (`validDeltaTMultiple`,
`kDeltaTMultipleTolerance`).

**Description**: The sim clock is a `double` accumulated by repeated
`+= dt`, then rounded to 1e-7 s on every step. When `dt` isn't exact in
binary (0.1, 0.2, 0.3, …), each step introduces a ~1e-16 rounding error;
over 1.8M steps (~50 h at dt=0.1) the accumulation reaches ~1e-11 s. That
is enough to trip `validDeltaTMultiple` if its tolerance is scaled by `dt`,
and it can drive other alignment checks close to failure with larger dt
or longer horizons.

**Impact**:
- Historically caused `CHECK failed: isValid` aborts in `advanceWorld`
  when running non-binary-exact `dt` for many hours (e.g. `physics_dt =
  0.1` on the week-long fleet sim).
- Currently mitigated by widening the tolerance to an absolute 1e-9 s
  (`kDeltaTMultipleTolerance`), which is enough headroom for realistic
  configs but doesn't remove the underlying drift.

**Related**:
- `stepWorld` line ~126: per-step `std::round(time * 1e7) / 1e7` — helps
  within a step but doesn't cure long-horizon drift because 1e-7-multiples
  aren't themselves exact multiples of a non-binary-exact dt.

**Proposed fix**: Store `simData.time` as `int64_t` nanoseconds (or a
similar fixed-point representation). Convert to `double` only at read
boundaries (sensor timestamps, log messages, config comparisons). Drift-
free arithmetic, no tolerance-tuning needed. Touches:
- `RbSimData` / `RbWorld` clock field and every read of it
- Every function that takes `double time` (advanceWorld, sensor updates,
  physics-loop scheduling in `main.cpp`, APRS `simTimeSec`, PfgWriter's
  timestamp fmts). Most can keep their `double` signature — only the
  internal clock changes.

**Workaround for users**: prefer binary-exact `physics_dt` values
(`0.125`, `0.25`, `0.5`, `1.0`, …) when running multi-day sims.

---

### [open] PfgWriter couples pose vertex cadence to `gt_freq_hz` instead of `odom_freq_hz`

**Location**: `mantaray/src/utils/PfgWriter.cpp:186-202` (and every downstream
call to `robotName(i, t)` in the same file).

**Description**: The `VERTEX_SE3:QUAT` writer iterates over the
`GroundTruthPose` sensor's timestamps to emit pose vertices. This couples
the pose graph's temporal discretization to the ground-truth sensor rather
than to the odometry sensor, which is inconsistent with the SLAM convention
that each odom step defines a new pose to estimate.

**Impact**:
- If a user sets `odom_freq_hz != gt_freq_hz`, pose vertex IDs won't align
  with odom cadence. Range-factor endpoints and GPS priors (which look up
  the nearest pose to associate with a measurement) will reference pose
  IDs that don't correspond to when odom actually sampled.
- The `use_ground_truth_odometry` flag currently affects only the odom
  edge value (GT-derived delta vs noisy-odom delta). Users reasonably
  expect it to also affect pose vertex cadence; it does not.
- Masked in current fleet-week configs because
  `gt_freq_hz == odom_freq_hz == 0.001`, so the two align 1:1.

**Related warning** (already in place): `validateSimConfig` in
`SimConfig.h` warns when `ping_interval` is not a whole multiple of
`1/gt_freq_hz`. Once this bug is fixed, that warning should reference
`odom_freq_hz` instead.

**Proposed fix**: Refactor five sections of `PfgWriter.cpp`:
1. Pose vertex writer — iterate over odom timestamps, pull pose values
   from nearest GT sample via `findNearestTimeIndex`.
2. First-pose prior — use odom's first timestamp with nearest GT value.
3. GPS pose prior — associate via odom timestamps instead of GT.
4. Odom edge writer (both GT and noisy branches) — index over odom cadence.
5. Range-factor endpoint helper (`robotGtTimestamps` -> `robotOdomTimestamps`).

Update section-header comments in each block to reflect that odom drives
pose graph structure and GT drives pose values.

---

## pymantaray/localization (Python solver + viz)

*(no open items)*

---

## pymantaray/acoustics (Python HYCOM + ray-trace tooling)

*(no open items)*

---

## Resolved

*(none yet — move closed items here with date + commit ref)*
