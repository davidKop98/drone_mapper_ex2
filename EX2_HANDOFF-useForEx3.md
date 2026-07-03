# EX2 Handoff — for EX3 / future sessions

This is the handoff written at the END of EX2 (July 2026), after the implementation was
finished, validated on the staff maps, and hardened by a full mutation-testing campaign.
It is the third document in the lineage:

- [DESIGN_NOTES-useInFutureEx.md](DESIGN_NOTES-useInFutureEx.md) — EX1 → EX2 handoff (EX1 architecture; mostly historical now)
- [EX2_MASTER_BRIEF.md](EX2_MASTER_BRIEF.md) — the orchestration brief EX2 was built FROM (schema probes, EX1 wisdom)
- **This file** — what EX2 actually became, what bit us, and what EX3 needs to know

Trust the code over any doc. Use this to know *where to look* and *why things are shaped
the way they are*. [readme.txt](readme.txt) holds the user-facing assumptions (gps-clamp
guard, fixture-isolation policy) and is part of the submission.

---

## 0. Hard environment rules (read before touching anything)

- **BUILD SINGLE-THREADED ONLY: `cmake --build build -j1`.** Parallel builds spike disk
  I/O to ~100% and crash the user's VS Code. This is a machine constraint, not a
  preference. It is also stored in Claude's auto-memory.
- Toolchain: C++20, GCC, `-Wall -Wextra -Werror -pedantic`, ninja, vcpkg
  (mp-units, TinyNPY, yaml-cpp, GTest/GMock).
- Full suite: `./build/drone_mapper_simulation_test` — **126 tests / 12 suites / ~82s**
  (the ~35s of that is the benchmark-map integration tests). Filters
  (`--gtest_filter='Map3D.*'`) are near-instant for component suites.
- `BENCHMARK_MAP_PATH` and `MAPS_COMPARISON_EXE_PATH` are compile definitions on the
  test target (CMakeLists.txt) — the second one is `$<TARGET_FILE:maps_comparison>` plus
  an `add_dependencies`, because two tests **spawn the real exe**.
- Staff skeleton remote workflow: add `https://github.com/Biddai/ex_2_skeleton` as a
  temporary `staff` remote, fetch, checkout *specific files only*, remove the remote.
  NEVER let a staff checkout clobber implemented files. `origin` is the user's repo
  (davidKop98/drone_mapper_ex2).

## 1. What EX2 is (30 seconds)

A batch drone-mapping simulator. A YAML "composition" expands to a cartesian product of
runs: per (simulation, its missions) group × drones × lidars. Each run: a drone with a
sphere-scanning lidar explores a hidden 3D voxel map (loaded from `.npy`), builds an
output map, which is scored against the hidden map (cell-accuracy %). Outputs:
`simulation_output.yaml` (report), `output_results/run_N/` (per-run maps),
`output_results/errors.txt` (immediate error log). A standalone `maps_comparison` exe
compares two `.npy` maps and prints ONLY the score to stdout (or `-1` on failure).

## 2. Architecture and ownership

| Piece | File(s) | Notes |
|---|---|---|
| Locked interfaces | `include/drone_mapper/I*.h` | **MUST NOT change** ("Do not change this interface"). We implement `*Impl`. |
| Course-provided | `ScanResultToVoxels`, `MockLidar`, TinyNPY (`NpyArray`) | Keep as-is-ish. MockLidar is "theirs" but WE are graded on testing it. |
| Map storage | `Map3DImpl` | THE single world→voxel snap site; uint8 encode/decode; everything depends on it. |
| Algorithm | `MappingAlgorithmImpl` | DFS state machine (see §3). |
| Step orchestration | `DroneControlImpl` | one `step()` = ask algorithm → move → scan → apply → map status. |
| Mission loop | `MissionControlImpl` | steps until Completed/Error/max_steps; always saves output map. |
| Batch | `SimulationManager` + `CompositionParser` | expansion, per-run isolation, report/yaml/errors.txt, CLI path rules. |
| Ground truth mocks | `MockGPS`, `MockMovement` | two-views-one-truth GPS; swept-sphere collision (see §3). |
| Scoring | `MapsComparison` + `maps_comparison_main.cpp` | sub-voxel sweep; exe contract. |
| Factory | `SimulationRunFactoryImpl` | loads hidden map, builds output map (all-Unmapped over mission bounds), resolves output resolution (Accepted/Ignored/IgnoredTooSmall), applies offset-relative start. |

## 3. Core design decisions (the "whole story" items)

These took real debugging to establish. Do not re-derive them from scratch.

**Two GPS views over one truth.** `MockGPS` wraps a shared `GpsTruth`. The *exact* view
(res ≤ 0) feeds MockLidar, MockMovement, and `ScanResultToVoxels::applyToMap` (map
writes). The *rounded* view (round-half-away-from-zero to nearest multiple of
`gps_resolution`; TA case 18@5→20) feeds the algorithm's `DroneState` — the drone plans
on *belief*, acts on *truth*. Heading is never rounded. Mapping is done in EXACT
coordinates; planning in ROUNDED ones. Leaking exact into the DroneState is the single
most design-central bug (pinned by `DroneControl.StateFedToAlgorithmIsRoundedView`).

**Collision responsibility is split.** The ALGORITHM avoids collisions via its clearance
rule: every planned center's footprint of `radius + gps_resolution/2` must be entirely
*confirmed Empty* — Occupied/PotentiallyOccupied/Unmapped/OutOfBounds all block (matches
EX1's rule; the gps/2 margin absorbs rounding so safe-as-believed can't crash as-true).
MockMovement is the ground-truth referee: bare radius, exact pose, hidden map,
swept-sphere sampling (0.5·res path step, per-axis box); reject = pose unchanged,
`{false, "DRONE_HITS_OBSTACLE"}`. A failed move is FATAL (DroneControl → Error → mission
Error → run scores −1). We once built and then **reverted** a "recoverable move /
MockMovement-does-the-checking" redesign — the user explicitly decided the algorithm owns
clearance and gps/2 stays. Don't resurrect it without asking.

**Coordinate conventions.** Heading 0°=east=+x, 90°=south=+y (clockwise convention, +Y
south — matches the Minecraft-style viewer axes); rotation Left = +angle, Right = −.
Advance moves `dist·(cos h, sin h)`; elevate is signed z. Voxel arrays are C-order
`[x][y][z]`, linear `x·ny·nz + y·nz + z`. Snap = `floor((world − offset)/res + 1e-9)`
per axis — the +ε makes an exactly-on-boundary coordinate land in the UPPER cell; the
lower-bounds check is written `!(fx >= 0.0)`-style ON PURPOSE so NaN rejects as
OutOfBounds instead of an undefined size_t cast (see §5, NaN chain).

**Offset semantics (staff clarified late).** `initial_drone_position` is RELATIVE to
`map_axes_offset` (real start = position + offset, applied in the factory).
Mission **boundaries are ABSOLUTE world coordinates**. The hidden map occupies world
`[offset, offset + shape·res)`.

**Voxel encoding.** 0=Empty, 255=Unmapped, 253=PotentiallyOccupied, anything else
nonzero decodes Occupied (hidden-map walls can be any byte, e.g. 9). OutOfBounds is
derived from extent, never stored. Encode: PO→253, Unmapped/OOB→255.

**The algorithm** (`MappingAlgorithmImpl`): fixed-direction DFS, phases
{Scanning, ChoosingNext, Moving, Backtracking, Finished}; ONE MappingStepCommand per
`nextStep` (one scan beam OR one capped move chunk). 10 candidate directions (8 compass
+ up/down), first-valid wins (E before S — test maps exploit this ordering). Backtracking
replays an inverse-move stack with level markers; a full unwind ends at the start pose.
Sphere sweep: xy 0..360, el −90..+90, step `clamp(2·atan(d/z_min)·deg, 4, 20)` — the
`·2.0` is a deliberate perf fix (≈¼ the beams) that got full-map runs under 1 minute
(34s, score 100). **Every** `drone_control_.step()` counts against `max_steps`,
including each scan beam (~180 beams/sphere at 20° step) — "scan counts as 1 step" was
assessed and is NOT a quick change (needs multi-command steps through the locked
interface).

**Scoring** (`MapsComparison`): sweep origin's MappingBounds at res/4 sub-steps, count
only samples in-bounds in BOTH maps, agreement = both-Occupied or both-not-Occupied
(Unmapped and PO are "not wall" — documented EX2 divergence from EX1's exact-match),
score = correct/total·100, `total==0` → −1 sentinel. Exe: bare number on stdout, −1 +
stderr text + nonzero exit on failure. Same-resolution only (different-res comparison =
the skipped bonus — a likely EX3 topic).

**Config guard:** `gps_resolution > map_resolution` desyncs belief from mapping by more
than a cell (drone freezes — clearance lands on Unmapped forever). Guarded in
`SimulationManager::runOne`: clamp to map res + log `GPS_RES_EXCEEDS_MAP_RES`
immediately. Documented in readme.txt.

## 4. Config schema (staff hierarchical format)

`sim_compose.yaml` → `simulation_compositions: { simulations: [{simulation_config:
<path>, mission_configs: [<paths>]}], drone_configs: [<paths>], lidar_configs:
[<paths>] }`. All refs relative to the compose file's directory. Referenced files carry
wrapper keys (`simulation_config:` etc.; parser also accepts bare blocks).
Fields: `dimensions_cm` is DIAMETER (radius = /2); `d_cm` = beam-circle spacing at
z_min; boundaries nested `{x,y,height}_boundary.{min,max}_cm`. CLI: no arg →
`simulation.yaml` in cwd; relative stays cwd-relative; absolute as-is
(`resolveCompositionPath`, unit-tested).

`inputs/` holds the staff's real configs (we FIXED their house files — see §5) and
`data_maps/benchmark_map.npy` (shape **(29,30,31)**, non-cubic — useful for axis-bug
testing) is the test-suite map. Scenario maps (`scenario_house/big/small.npy`): 150cm
solid ground occupying map z0-14 (world z[150,300] at offset z=150), building above.

## 5. Wars we fought (so you don't refight them)

- **"Large drone scores same as small"** — not a bug: full-bounds scoring is diluted by
  ~13k buried ground voxels (caps ~44); house-concentrated bounds showed both plateau
  because of step-budget starvation, not logic. Run-to-completion scored 100.
- **Staff maps "frozen drone"** — the staff's original configs spawned the drone flush
  against walls (start coords on cell boundaries = wall faces, radius embedded). We
  proved offset/axis/decode were correct via probes; staff later fixed their files.
  **Known edge:** a drone spawned in/against a wall silently reports **Completed**
  (~339 steps: one sphere scan, no legal move, empty stack → Finished). No error is
  logged. Adding a start-pose-vs-hidden-map check in the factory was proposed but never
  ordered — candidate EX3 hardening.
- **House configs** — we fixed `inputs/` ourselves: start `height_cm: 175` (real z =
  175+150 = 325, floor-1 interior), full-mission z-bounds [300,430], lower [300,350].
  After the fix: 24/24 staff-config runs clean, scores 62–94 by scenario.
- **NaN pose chain** — a mutated/degenerate GPS (res=0 dividing) produced NaN positions
  that **segfaulted the whole test binary** (NaN passes `< 0` and `>= n` checks → UB
  cast → wild index). Two intentional guards now in product code:
  `MockMovement::sweptSphereHitsOccupied` rejects non-finite distance;
  `Map3DImpl::snapToVoxel` lower-bound check is NaN-rejecting. Keep them.
- **FP snapping** — cell centers are `k·res + res/2` everywhere in tests; the ±1e-9 snap
  ε matters for real drift (pathSafe midpoints like 49.999999999999). The ε is pinned
  from both sides (removal, sign-flip, 1e-3 inflation all caught).

## 6. The mutation-testing campaign (the bulk of the grade)

**Grading model:** the staff inject a single-line bug into one component and run OUR
suite. Per injection: the component's own suite must fail ≥1 test (≥60% of injections
caught), integration must catch ≥20%, and suites of UNAFFECTED components must NOT fail.
Suite↔component map: each `*Impl` has its suite; **MockMovement/MockGPS are graded via
`SimulationRun.*`**; MockLidar has `MockLidar.*`; integration = `Integration.*` +
`Radical.*`.

**Protocol used (keep for EX3):** per component — apply one realistic single-line
mutation → full suite → record (component / integration / other-suite failures) → revert
→ verify clean tree → next. Then write targeted tests for survivors and **re-verify:
test added → mutation re-applied → red → revert**. "New test passes on clean code" is
NOT acceptance; the re-verified red is.

**Final scorecard** (~105 mutations measured; catch-rates exclude agreed equivalents):

| Component | Suite | Final | Baseline | The gap that was found |
|---|---|---|---|---|
| MappingAlgorithmImpl | MappingAlgorithm.* | 11/12 | 8/12 | backtracking entirely unobserved (pose-tracking harness + closed T-map fixed it); sweep coverage |
| Map3DImpl | Map3D.* | 12/12 | 10/12 | per-axis offsets never distinct; C-order contract untested (set→atVoxel round-trips are self-consistent under ANY index bijection — must pin against hand-computed raw bytes) |
| MockMovement+MockGPS | SimulationRun.* | 14/14 | 12/15 | radius term never load-bearing (all wall tests drove the centerline in) |
| MapsComparison(+exe) | MapsComparison.* | 13/13 | 9/14 | PO-in-target unpinned; nonzero-min-bounds alignment; **exe contract had zero tests** |
| DroneControlImpl | DroneControl.* | 14/14 | 10/14 | argument blindness — GMock `_` wildcards everywhere (fixed with captures) |
| MissionControlImpl | MissionControl.* | 9/9 | 8/9 | error message content unasserted |
| SimulationManager+Parser | SimulationManager.* | 14/14 | 8/15 | summary/yaml-fields/log-content/per-run-dirs/parser-keys all unread by tests |
| MockLidar | MockLidar.* | 12/12 | **4/12** | fixture symmetry: everything at heading 0, altitude 0, fov≤2; fan structure never asserted |

**Equivalents ledger** (documented as uncatchable/uncontracted; don't chase, and be
ready to defend to the TA): M2 (algorithm "Unmapped traversable" — masked by the
triple-layer veto isEmptyCell+pathSafe+hasClearance), V12 (MockMovement collide-on-PO —
no 253 byte on any swept hidden path), C11 (compare multi-target break — nothing passes
>1 target anywhere), G3 (drone/lidar loop-order swap — run order is uncontracted).

**Cross-suite isolation (crucial for the "unaffected suites" rule):**
- `tests/components/FakeMap3D.h` — hand-rolled IMutableMap3D (sparse cells, own snap,
  `unset` default arg). Used by MockLidar/DroneControl/MappingAlgorithm tests.
- `tests/components/FakeGps.h` — hand-rolled IGPS (fixed pose + setters). Used by
  MockLidar/DroneControl/SRV-parity tests.
- Suites that deliberately keep REAL dependencies (subject matter, defensible):
  ScanResultToVoxels + MapsComparison on real Map3DImpl; SimulationRun factory tests
  (Map3DImpl is the factory's product); MissionControl save tests (save path).
  Documented in readme.txt.
- Two seam tests live in the **Integration** suite by design (a catch there is credit,
  never a penalty): `Integration.MovementParityWithAlgorithmClearance` (physically in
  SimulationRunTest.cpp — suite name is what grading keys on).
- Empirically **zero cross-suite bleed** under every mutation after this work.

**Recurring mechanics worth remembering:**
- `-Werror` blocks three whole grader-mutation classes: unused-variable (can't just
  delete a term — zero its coefficient instead), unused-parameter (can't drop a field
  write), non-exhaustive switch (can't delete a case — move the label). Naive mutations
  DON'T COMPILE; always build before trusting a run (a failed build silently runs the
  stale binary!).
- Assertion-style lessons: **exact-value** tests (99.2, 87.5, 26/27...) catch
  region/sampling/off-by-one bugs that property tests (identical→100, inverted→0) are
  blind to; **sentinel** tests carry the guard paths alone; **argument capture** beats
  GMock `_` wildcards (the whole DroneControl gap class); fixtures must be
  **asymmetric** (distinct value per axis/field/count so a crossed wire can't coincide —
  e.g. exact (252,173,91) vs rounded (250,175,90); drone yaml 11/22/33; group missions
  2-vs-3).
- Clever tricks that worked: the mock factory's *second* `create()` reading errors.txt
  **mid-batch** (pins immediate-vs-deferred logging, which post-hoc reads can't see);
  a pose-tracking harness that applies emitted movement commands with MockMovement's
  exact semantics to observe backtracking; closed all-Occupied maps with carved Empty
  arms so a complete DFS must end plain `Finished`; spawning the real exe with stdout
  parsed as "one number and nothing else".
- Mutations that segfault kill the WHOLE gtest run (all later tests unreported) — worse
  than failing. That motivated the NaN guards.

## 7. Current state (as of this handoff)

- All campaign work is committed: `5b24e24 "all components tested"` on `main`
  (tests, FakeMap3D/FakeGps fixtures, the two NaN guards in src, CMake wiring,
  readme.txt).
- **⚠ SUBMISSION LANDMINE: `tests/smoke_test.cpp` is UNTRACKED but listed as a source
  in CMakeLists.txt** — a fresh clone of the repo will fail to configure/build. Track it
  (or drop it from CMake) before any submission or grader checkout.
- Still untracked besides that: the three handoff/brief .md docs and `ex1_reference/`
  (fine to leave out of the submission, but decide deliberately).
- Suite: 126/126 green, ~82s single-threaded.
- Claude auto-memory (`~/.claude/projects/.../memory/`) holds the campaign log
  (`mutation-testing-campaign.md`) and the build-single-threaded rule.

## 8. EX3 briefing — TA hint: algorithm efficiency + concurrent execution

The TA has hinted EX3 focuses on **making the algorithm more efficient** and on
**concurrent running (mutexes etc.)**. Here is the prepared ground, in that order.

### 8a. Algorithm-efficiency backlog (known inefficiencies, already diagnosed)

Ranked by expected payoff; all were observed/measured during EX2, none are speculative:

1. **The sphere sweep is the step hog.** ~180 beams per visited cell at the current 20°
   step (already a deliberate 2×-widening; it took the full benchmark run 125s → 34s at
   score 100). Two concrete wins sitting untouched:
   - **Per-direction scan gating**: we scan ALL directions unconditionally; skipping
     cones whose target region is already mapped (no Unmapped along the cone) would cut
     most rescans. The mutation campaign explicitly noted "no per-direction gating
     exists" — `MappingAlgorithm.SweepCoversFullElevationRange` asserts every sweep
     spans el −90..+90 / xy 0..340, so **gating will break that test by design**; relax
     it to "first sweep is full" or assert gated coverage semantics instead.
   - **Zenith duplicates**: the el=+90 ring emits ~18 beams that all point straight up
     (same for −90). Pure waste, trivially dedupable.
2. **Backtracking physically retraces every level** (rot 180 → advance → rot −180, per
   level, in capped chunks). Replacing unwind-by-replay with **path-planning through
   known-Empty space to the nearest frontier** (BFS/A* over the visited/Empty cell
   graph) is the classic upgrade and would slash step counts on deep trees. If you do
   this, `BacktracksOutOfDeadEndAndMapsSecondBranch` / `BacktrackReplaysInverseMoves...`
   pin the CURRENT contract (physical re-traversal, return-to-start) — they must be
   consciously rewritten to the new contract (completeness + continuity, not replay).
3. **Clearance is the CPU hot loop**: `hasClearance` samples an O(n³) box per candidate
   and `pathSafe` calls it per path sample, recomputed from scratch every time. A cached
   per-cell "clear" map invalidated on map writes, or incremental distance-field, is the
   obvious optimization. All map reads are `atVoxel` — read-only, so this also
   parallelizes trivially (see 8b).
4. **Frontier selection is naive** (fixed direction order, first-valid). Nearest-frontier
   selection combines naturally with #2's pathfinding.
5. **"Scan counts as 1 step"** — if EX3's efficiency metric is steps, this returns; it
   was costed in EX2 as NOT quick (one `nextStep` = one command through the locked
   interface). If EX3 unlocks/changes interfaces, revisit first.

Measurement harness: the benchmark integration tests (`RunSpec` in
BenchmarkMapIntegrationTest.cpp) already parametrize map/bounds/drone/lidar and report
score+steps — that's your before/after efficiency rig. Keep score thresholds while
driving steps down.

### 8b. Concurrency — shared-state inventory (audited during EX2)

**The natural parallel axis is SimulationManager's run loop** — runs are independent by
design (that was the whole isolation invariant). What is actually shared today:

| Shared thing | Today | Under threads |
|---|---|---|
| `errors.txt` (single `std::ofstream` passed into every run) | sequential writes, flushed immediately | needs a mutex around `logError`; keep write+flush atomic per line. **The "immediate, not deferred" PDF rule survives**, but our `ErrorsTxtImmediateWithCodeAndMessage` test observes immediacy via *another run's* create() — under concurrency that ordering is racy; rewrite it to a same-thread observation or a completion-latch. |
| `runs` vector + `index` counter | `push_back` in loop order | **pre-size the vector and write by index** (each run's slot computed from its position in the cartesian product). This preserves deterministic report order with zero locking — and it settles G3: if run order becomes contracted, add the order-sensitive expansion test then. |
| Hidden map | loaded from disk **per run** by the factory | share one loaded `NpyArray`/`Map3DImpl` per group — big IO/memory win AND safe: `Map3DImpl::atVoxel`/`isInBounds` are read-only (`set()` is NOT thread-safe; output maps are per-run so that's fine). |
| Per-run state (output map, GpsTruth, drone stack) | one per run | already isolated; no sharing, no locks needed. |
| `run_N` dirs / output files | created per run | already distinct paths (pinned by `PerRunOutputDirsAreDistinct`); safe. |

Within-run parallelism (secondary, easy because read-only): MockLidar's beams are
embarrassingly parallel over a const map; MapsComparison's sweep is a pure reduction
over two const maps; clearance sampling likewise. The only per-run writer is the
converter into the output map (single writer — keep it single-threaded per run).

**Cautions specific to this machine/project:**
- The `-j1` BUILD rule is about build-time disk I/O and still applies; runtime threads
  are fine, but N concurrent runs all writing `run_N` maps can spike disk — keep batch
  parallelism modest (or make thread count configurable) on this box.
- gtest runs everything in one process: a data race can poison unrelated suites. Keep a
  `threads=1` mode so the deterministic suite stays deterministic, and add a dedicated
  concurrency test: run the same composition sequentially and with N threads, assert the
  two REPORTS are equal as sets (score/config tuples) — the set-identity style from the
  campaign (G1/G2 fixes) is exactly the right assertion shape here, since order is the
  one thing threading legitimately changes.
- Races are not single-line-mutation material, but "forgot the mutex" IS a plausible
  grader injection once a mutex exists — a stress test (many tiny runs, all errors
  logged, assert line count exact) catches missing-lock corruption most reliably.
  ThreadSanitizer exists but is a separate build config; probably overkill unless
  grading demands it.

### 8c. Still-open smaller items (pre-hint list, kept for completeness)

- **Different-resolution comparison** (skipped EX2 bonus): the exe already accepts an
  ignored `comparison_config=<path>` arg; compare()'s sub-voxel sweep was designed for
  misaligned grids.
- **Multi-target compare** is unexercised (>1 target — C11); write the 2-target test
  before relying on it.
- **Spawn-in-wall detection** (factory check + errors.txt entry) — scoped, never ordered.

### 8d. Method

Whatever EX3 adds, **reuse the campaign protocol** (§6): sweep single-line mutations →
find survivors → targeted exact-value tests → re-verified red. It converted every EX2
suite from "looks tested" to provably mutation-catching, revealing 2–8 real gaps per
component. For the algorithm rework specifically: the pose-tracking harness + closed
T-map in MappingAlgorithmTest.cpp is the template for testing ANY new exploration
strategy (completeness, continuity, terminal status) without touching the real sim
stack — port it to the new algorithm before optimizing, so efficiency work happens
under a green safety net.
