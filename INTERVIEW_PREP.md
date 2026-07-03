# EX2 Interview Prep — project summary with a deep-dive on the tests

Personal study document. The implementation largely mirrors EX1's ideas, so it is
summarized briefly; the tests are where this project's real work went and where the
detail is. For every non-trivial test: *what it checks* and *why it was added*.

---

## 1. The project in two paragraphs

EX2 is a batch drone-mapping simulator. A YAML "composition" expands into a cartesian
product of runs — per (simulation map + its missions) group × drones × lidars. In each
run, a drone with a sphere-scanning lidar explores a hidden 3D voxel map (`.npy`),
builds an output occupancy map, and gets scored by cell-accuracy against the hidden map.
The batch writes `simulation_output.yaml`, per-run maps under `output_results/run_N/`,
and an immediately-flushed `errors.txt`. A standalone `maps_comparison` executable
compares two maps and prints only the score (or `-1` on failure).

Architecture: locked `I*.h` interfaces (untouchable) with our `*Impl` classes behind
them — `Map3DImpl` (voxel storage, THE single world→voxel snap site), `MappingAlgorithmImpl`
(DFS explorer), `DroneControlImpl` (one step = ask algorithm → move → scan → apply),
`MissionControlImpl` (step loop until Completed/Error/max_steps), `SimulationManager` +
`CompositionParser` (batch), `MockGPS`/`MockMovement` (ground truth), `MapsComparison`
(scoring). `ScanResultToVoxels` and `MockLidar` are course-provided.

## 2. Design decisions worth being able to explain

- **Two GPS views over one truth.** The exact view feeds the lidar, the movement, and
  map-writing; the rounded view (round-to-nearest multiple of `gps_resolution`) feeds
  the algorithm's `DroneState`. The drone *plans on belief, acts on truth*. Mapping is
  in exact coordinates, planning in rounded ones.
- **Split collision responsibility.** The algorithm is responsible for never planning a
  collision: it requires a footprint of `radius + gps_resolution/2` to be entirely
  *confirmed Empty* (Occupied / PotentiallyOccupied / Unmapped / OutOfBounds all block).
  The gps/2 margin exists so a move that is safe-as-believed cannot crash as-true.
  MockMovement is the referee: bare radius, exact pose, hidden map, swept-sphere check;
  a rejected move changes nothing and is fatal to the run (scores −1).
- **DFS state machine**, one command per `nextStep` (a single scan beam OR a single
  capped move chunk). Backtracking replays an inverse-move stack with level markers.
  Sphere sweep at every visited cell; scan angle step widened 2× as a deliberate
  perf tradeoff (full benchmark map: 125s → 34s, still score 100).
- **Every `step()` counts against max_steps** — including each of the ~180 scan beams
  per sphere. This dominates the step budget (relevant if asked about efficiency).
- **Scoring**: sub-voxel sweep (res/4) of the origin's mission bounds, counting only
  samples in-bounds in BOTH maps; binary Occupied-vs-not agreement; Unmapped and
  PotentiallyOccupied count as "not wall"; no comparable samples → −1 sentinel.
- **Config guard**: `gps_resolution > map_resolution` breaks the belief/act consistency
  (the drone freezes — clearance forever lands on Unmapped). We clamp to map resolution
  and log `GPS_RES_EXCEEDS_MAP_RES` immediately.
- **Offset semantics**: `initial_drone_position` is relative to `map_axes_offset`
  (real = pos + offset); mission boundaries are absolute world coordinates.
- Conventions: heading 0°=+x, 90°=+y (clockwise), Left=+; C-order `[x][y][z]`;
  snap `floor((world−offset)/res + 1e-9)` — the +ε puts exact-boundary points in the
  upper cell; encoding 0/1(or any nonzero)/253/255 = Empty/Occupied/PO/Unmapped.

## 3. The testing story (the headline)

The grade rides on the tests: the graders inject a **single-line bug** into one
component and run our suite. Per injection, the buggy component's own suite must fail
at least one test (≥60% of injections must be caught), the integration suite must catch
≥20%, and suites of *unaffected* components must not fail.

So we didn't just "write tests" — we ran the graders' game against ourselves,
**mutation testing**, component by component:

1. Apply one realistic single-line mutation (the kind a grader would inject).
2. Run the full suite; record which suites fail.
3. Revert, verify a clean tree, repeat (~105 mutations across 8 components).
4. Every mutation that *survived* is a proven blind spot → write a targeted test.
5. Acceptance criterion: **re-apply the mutation and watch the new test go red** —
   "the new test passes on clean code" proves nothing.

Every component ended at 100% of catchable mutations:

| Component | Suite | Final | Baseline | The blind spot the campaign exposed |
|---|---|---|---|---|
| MappingAlgorithm | MappingAlgorithm.* | 11/12 | 8/12 | backtracking completely unobserved |
| Map3DImpl | Map3D.* | 12/12 | 10/12 | memory-layout & per-axis-offset contracts untested |
| MockMovement+MockGPS | SimulationRun.* | 14/14 | 12/15 | the collision radius was never load-bearing |
| MapsComparison + exe | MapsComparison.* | 13/13 | 9/14 | exe contract had zero tests; frame alignment |
| DroneControl | DroneControl.* | 14/14 | 10/14 | argument blindness (GMock wildcards) |
| MissionControl | MissionControl.* | 9/9 | 8/9 | error *content* unasserted |
| SimulationManager+Parser | SimulationManager.* | 14/14 | 8/15 | outputs written but never re-read |
| MockLidar | MockLidar.* | 12/12 | **4/12** | fixture symmetry hid 2/3 of the surface |

Suites of unaffected components fail **zero times** under any mutation — that took
dedicated isolation work (§4).

**Equivalents ledger** (mutations no test can catch; documented and defensible):
- *Algorithm, Unmapped-traversable*: masked by a triple-layer veto (target check, path
  re-check, footprint check) — relaxing any single layer is vetoed by the next.
- *MockMovement, collide-on-PotentiallyOccupied*: no hidden-map fixture has a 253 byte
  on any swept path; arguably defensible physics anyway.
- *Compare, multi-target break*: nothing in tests or production ever passes >1 target.
- *Manager, drone/lidar loop-order swap*: same run set, different order; order is
  uncontracted.

**A build-system fact worth mentioning**: `-Werror` blocks three whole classes of naive
mutations — unused-variable (can't delete a term; zero its coefficient), unused-parameter
(can't drop a field write), and non-exhaustive switch (can't delete a case; move the
label). Several "obvious" injections simply don't compile.

## 4. Test infrastructure (each piece exists for a reason)

- **`FakeMap3D`** (hand-rolled `IMutableMap3D`: sparse cell map, its own trivial snap,
  constructor extents). *Why:* Map3D mutations were failing MockLidar/DroneControl/
  MappingAlgorithm suites — those suites used real `Map3DImpl` as scaffolding, which
  violates the "unaffected suites must not fail" rule. A hand-rolled fake (NOT a copy of
  Map3DImpl's arithmetic, not a GMock) means no Map3DImpl bug can propagate. Suites
  whose *subject matter is* the map (ScanResultToVoxels, MapsComparison) deliberately
  stay on the real one — a fake would test nothing there.
- **`FakeGps`** (fixed pose + setters, zero shared code with MockGPS). *Why:* same
  story — a MockGPS mutation (shared-truth constructor flip) failed MockLidar and
  DroneControl. SimulationRun keeps the real MockGPS because that suite *grades* it.
- **Pose-tracking harness** (`runWithPose` in MappingAlgorithmTest): applies every
  movement command the algorithm emits using MockMovement's exact semantics (Left=+,
  advance along cos/sin) and feeds the updated state back. *Why:* all the older
  algorithm tests fed a frozen DroneState, so exploration *dynamics* (backtracking,
  rescans) were invisible — two mutations that corrupted backtracking survived a
  100-test suite while making it run visibly faster.
- **Closed T-map fixture** (`fillTee`): everything Occupied except two carved Empty
  arms. *Why:* with no Unmapped anywhere, a complete exploration MUST end in plain
  `Finished` — turning "did it really explore everything?" into a crisp assertion.
- **Real-process exe tests**: CMake passes `$<TARGET_FILE:maps_comparison>` as a
  compile definition (plus a build dependency); tests spawn it via `std::system`,
  redirect stdout/stderr to files, take `WEXITSTATUS`. *Why:* the PDF's exe contract
  ("only the number on stdout", "−1 on failure") is exactly what a refactor-to-function
  can't fully pin — stdout purity is a process-level property.
- **Mid-batch log observation**: the mocked factory's *second* `create()` call reads
  `errors.txt` while the batch is still running. *Why:* the PDF requires errors logged
  *immediately, not deferred* — any test that reads the log after the run finishes
  can't tell the difference (the stream flushes on close); an in-flight observer can.
- **Two seam tests intentionally live in the Integration suite**
  (`Integration.MovementParityWithAlgorithmClearance`): they use two real components at
  once, so in ANY single component's suite they'd fail under the *other* component's
  bug. In Integration, a catch is credit, never a penalty.

**House style rules that came out of the campaign** (say these if asked "how do you
write good tests"):
- *Exact values over thresholds*: tests asserting 87.5 / 99.2 / 26 of 27 caught
  region-sampling and off-by-one bugs that "identical→100, inverted→0" property tests
  were completely blind to. Keep all three styles (exact, property, sentinel) — each
  was the sole catcher for some mutation class.
- *Asymmetric fixtures*: every axis/field/count distinct (position (252,173,91) vs
  rounded (250,175,90); drone yaml 11/22/33; groups with 2-vs-3 missions) so a crossed
  wire can never coincidentally produce the right answer.
- *Capture arguments, don't wildcard them*: GMock's `_` verifies a call happened but
  not what went in — an entire class of DroneControl bugs (wrong GPS view, dropped
  heading, dropped scan orientation) lived in wildcarded arguments.

## 5. Per-component test walkthrough

Trivial tests get one line; the interesting ones get what+why.

### Map3D.* (10) — voxel storage
Basics in one line: `SnappingAt1cm/10cm` (cell bucketing), `IsInBounds`,
`OutOfBoundsReturnsOutOfBounds`, `NegativeOffsetSnapsIntoRange` (negative world coords).

- **DecodesRawBytes** — plants raw bytes {0,1,9,253,254,255} and reads through
  `atVoxel`: 9 must read Occupied (hidden walls can be any nonzero), and **254** must
  read Occupied. *Why 254:* it is one off from BOTH sentinels — pins 253 and 255
  against ±1 drift with a single value; before it, a 253→254 constant mutation was
  caught by exactly one assert.
- **BoundaryCoordLandsInUpperCell** — three-sided ε test: exactly-on-boundary → upper
  cell (catches sign-flip), `nextafter`-below-boundary → upper cell (catches removal),
  and 29.995 stays in the lower cell (catches ~1e6× inflation). *Why:* the snap ε is a
  single constant with three distinct failure modes; one assert only covered one.
- **SaveReloadRoundTrip** — set a pattern incl. PO, save, reload, compare all voxels,
  **plus assert the raw stored byte is literally 253**. *Why the raw byte:* a symmetric
  encode/decode corruption (both sides using a wrong constant) round-trips cleanly and
  hides; only reading the physical byte breaks the symmetry.
- **DistinctPerAxisOffsetSnapsCorrectly** — map with offset (10,20,30) — different on
  every axis; hand-planted raw byte at cell (2,3,4) read back via world coords, and the
  reverse (set → hand-read the byte). *Why:* the "z uses y's offset" mutation survived
  the whole suite — every older fixture used offset (0,0,0), where all axis mix-ups are
  invisible. Raw-byte verification is essential: a set/atVoxel pair sharing the same
  broken snap round-trips to a false pass.
- **RawByteAtNonzeroIndexReadsThroughAtVoxel** — non-cubic 2×3×4 grid; byte planted at
  hand-computed C-order index 23 = (1,2,3); exhaustive sweep asserts every OTHER cell is
  untouched; plus the inverse (set → check the byte lands at index 14). *Why:* the
  linear-index formula (`x·ny·nz + y·nz + z`) was effectively untested — set→atVoxel
  round-trips are self-consistent under ANY index bijection, older raw-byte tests used
  x=0 or a 5×1×1 grid where wrong formulas coincide, and all grids were cubes where
  swapped dimension factors are equal. This is the memory-layout *contract* test.

### SimulationRun.* (16) — MockGPS + MockMovement + factory (the mocks' graded suite)
Basics: 4 GPS tests (round-to-nearest incl. the TA's 18@5→20 case and negatives; exact
view raw; heading never rounded; **GpsSharedTruthSingleUpdateBothViews** — write through
one view, both views see it: the two-views-one-truth contract), 4 factory tests (hidden
map load/offset, output map all-Unmapped over bounds, resolution status), clear-advance
updates both views, elevate down-then-ceiling-crash.

- **AdvanceThroughThinWallBlocked / AdvanceEndpointInWallBlocked /
  FailedAdvanceCommitsNothing** — a deliberate three-way split of what used to be one
  test. *Why:* one overloaded test single-handedly caught three different mutations
  (endpoint-only collision check, coarse march step, commit-despite-failure) — one
  assertion-depth from becoming three survivors. The thin-wall geometry is engineered so
  a 2×-resolution march *straddles* the wall (mutated sampling boxes at [13,17]/[33,37]/
  [53,57] all miss the wall cell [20,30)) while the correct half-cell march samples
  inside it; the endpoint stays clear so an endpoint-only check passes the move.
- **AdvanceBlockedWhenWallClipsRadiusOnly** — the flagship geometry test. The wall never
  touches the movement centerline: it sits at lateral distance 5cm, with radius 8 —
  chosen so r/2 = 4 < 5 < 8 = r. Honoring the full radius hits it; a zeroed OR halved
  radius sails through. A control wall at 15cm (> r) must NOT block — pinning the radius
  from above too. *Why:* both "radius zeroed" and "radius halved" mutations survived the
  entire suite — every wall test drove the center of the drone into the wall, so the
  radius term was never load-bearing. One test, radius pinned from both sides.
- **RotateDirectionSign / RotateNeverCollidesEvenTouchingWall** — split pair: Left/Right
  from heading 90 produce 120/70; and rotating with the sphere fully EMBEDDED in
  Occupied space must succeed both ways. *Why the second:* a sphere is rotationally
  symmetric, so rotation must never collide — a mutation adding a stationary collision
  check to rotate() survived until this test placed the drone inside a wall.

### MockLidar.* (9) — worst baseline of the campaign (4/12): fixture symmetry
Basics: `StraightScanHit` (hit at 95±1), `MissReturnsMaxDouble`, `TooCloseReturnsZero`,
`HitsCarryRelativeAngles` (center-beam angle excludes heading),
`UsesExactPositionNotRounded` (traces from exact 252, not rounded 250 → 48cm not 50cm).

- **ScanAtHeading90HitsWallInY** — drone facing +y, wall in +y, hit at exactly 95cm,
  relative angle 0. *Why:* dropping the sensor heading from the beam direction survived
  the suite — every distance-asserting scan fired at heading 0, where scan-only and
  scan+heading are identical.
- **BeamFanStructureAndSpread** — fov_circles=5 → exactly **341** beams (1+4+16+64+256);
  entry 0 is the center at the raw scan orientation; circle-1 θ=0 beam offset is exactly
  atan2(d, z_min)=45°, circle-2 exactly atan2(2d, z_min); all at a NONZERO heading that
  must not leak into the (relative) angles. *Why:* four separate mutations survived
  because the fan's structure had zero assertions — beam count per circle (×4 growth),
  radius growth per circle (k·d), the d-at-z_min spacing definition, and the
  relative-angle contract for ring beams (only the center beam was ever checked). One
  structural test kills all four.
- **HitExactlyAtRangeBoundaries** — two scenarios where the first in-wall sample lands
  *exactly* at z_max (still a hit — `<=`) and exactly at z_min (NOT too-close — strict
  `<`; returns 95, not 0). *Why:* both boundary-comparison mutations (`<=`↔`<`) were
  full survivors — no fixture ever hit exactly on a range boundary. The geometry (origin
  5, wall face 100, boundary 95) makes the boundary sample exact in FP (1cm integer
  steps).
- **AltitudeBeamHitsCeilingAtExactDistance** — 45°-up beam; the ceiling cell is first
  sampled at exactly d=36 (hand-derived from the cell faces at x≥50 ∧ z≥30 with
  direction (0.7071, 0, 0.7071)). *Why:* dropping the cos(altitude) scaling of the
  horizontal components survived — every older test fired at altitude 0 where cos=1.
  The mutated (unnormalized) direction overshoots in x and misses the cell entirely.

### ScanResultToVoxels.* (7) — course-provided converter (kept on real Map3DImpl)
Mostly behavioral pins of the writing rules: ray marked Empty + endpoint Occupied;
miss marks Empty to z_max; too-close marks PotentiallyOccupied to z_min; **stickiness**
(an Occupied voxel survives a later Empty ray — sensor disagreement resolution);
PO never overwrites Occupied; out-of-bounds samples not written.
**LidarToConverterToMapParity** chains real MockLidar → converter → map and asserts the
wall voxel lands in the right cell — the lidar↔converter geometric-frame agreement test.

### MappingAlgorithm.* (15) — the DFS
Basics: bootstrap emits a scan not a move; frontier advance only into confirmed-Empty;
non-traversable neighbors never entered; PotentiallyOccupied treated as wall (control
pair: same corridor, Empty entered / PO not); Occupied inside the footprint blocks an
otherwise-clear move; per-command rotate/advance caps respected; determinism (same
inputs twice → identical command); Finished vs FinishedWithUnmappableVoxels terminal
statuses.

- **FootprintNeverOverlapsUnmapped** — the central safety invariant: a target that is
  itself Empty but whose radius+gps/2 footprint touches Unmapped is never entered.
  *Why:* this IS the collision-avoidance design; three separate clearance mutations
  (dropped margin, always-pass, Unmapped-not-blocking) are all caught here.
- **BacktracksOutOfDeadEndAndMapsSecondBranch** — pose-harness + T-map: drive the real
  algorithm to the east dead-end; assert it physically reaches the south branch too and
  terminates `Finished`. *Why:* the "quit at first dead-end" mutation survived 100
  tests — nothing ever verified exploration *continues past* a dead-end. (Telling
  detail: under the mutation the whole suite ran 57s instead of 83s — the drone was
  demonstrably exploring less, and no assertion noticed.)
- **BacktrackReplaysInverseMovesFromCurrentPosition** — same map; asserts every implied
  position stays in carved-Empty space, the corridor is re-entered ≥2 times on the way
  out (no teleporting), and the full DFS unwind returns the believed position to the
  start cell. *Why:* the backtrack sentinel-flip mutation made "backtracking" a
  bookkeeping no-op (position never physically moved back) and survived — positional
  continuity was unobserved.
- **SweepCoversFullElevationRange** — collects the scan orientations of EVERY sweep
  (including rescans after moves) and asserts each spans el −90..+90 and xy 0..340.
  *Why:* a mutation making rescans start at el=0 (never scanning downward again) was
  invisible to the component suite — only one integration score noticed. Asserting all
  sweeps covers both the rescan bug and the first-scan-initializer variant.
- **ChunkedAdvancesSumToRequestedDistance** — collects the whole chunk stream for a
  10cm move under a 3cm cap: each ≤ cap, exactly 4 chunks, sum exactly 10. *Why:*
  the cap-drop mutation was caught by exactly one assert (≤ cap); the *sum* clause
  catches the complementary corruption family (wrong remainder bookkeeping) that the
  cap check alone misses.

### DroneControl.* (8) — step orchestration (GMock'd collaborators + fakes)
Basics: movement executes before the scan (InSequence); failed movement → Error, scan
skipped (`Times(0)`); AlgorithmStatus→DroneStepStatus mapping; the taken scan is
threaded into the NEXT nextStep (IsNull on first call, NotNull + captured content on
the second).

- **AppliesScanFromExactOrigin** — exact GPS 252 vs rounded 250 (res 5); the scan hit
  48cm ahead must land in cell 30 (252+48=300), not 29. *Why:* the highest-stakes line
  in the class — applying scans from the rounded position smears the whole map; this
  test distinguishes the two views by 2cm across a cell boundary.
- **StateFedToAlgorithmIsRoundedView** — captures the DroneState passed to the mocked
  algorithm across two steps; position must equal the ROUNDED view on all three axes
  (fixture: exact (252,173,91) vs rounded (250,175,90), distinct on every axis), and
  step_index must go 0 → 1. *Why:* feeding the algorithm the exact view — which
  silently breaks the entire GPS-resolution design — survived everything: GMock
  wildcards never inspected the argument, and integration *tolerates* the bug because
  exact information makes the drone better, not worse. The step_index pin came free
  with the same capture.
- **AppliesScanFromExactOriginAtHeading90** — the heading-asymmetric twin: at heading
  90 the same 48cm hit must land in +y. *Why:* zeroing the heading passed every test
  because all fixtures faced heading 0; the pair (0° and 90°) proves the *current*
  heading is threaded through, not a constant.
- **ScanCommandOrientationPassedToLidar** — the algorithm commands a scan at (37°,−12°)
  (non-zero, distinct per axis); capture what `lidar.scan()` receives and assert both
  fields. *Why:* every `EXPECT_CALL(lidar, scan(_))` wildcarded the orientation — a
  "always scan at default orientation" mutation was component-invisible.

### MissionControl.* (4) — the step loop (best baseline: 8/9)
All four are load-bearing; this suite was written exact-value from the start:
- **CompletedWhenDroneFinishes** — Continue, Continue, Completed → status Completed,
  steps EXACTLY 3, map file saved. The exact step count catches budget/counter/reporting
  corruption (three different mutations died on the `== 3`).
- **MaxStepsWhenNeverFinishing** — never-finishing mock, cap 5 → MaxSteps, steps
  EXACTLY 5 (catches `<`→`<=` off-by-one and the status-init swap).
- **ErrorShortCircuits** — error at step 2 → Error, steps 2, and (added by the campaign)
  `errors[0].code == "DRONE_STEP_ERROR"` and `message == "DRONE_HITS_OBSTACLE"`.
  *Why the addition:* blanking the message survived — the old assert only checked
  errors *exist*; "an error happened" and "the error detail is carried" are different
  contracts.
- **SavesOutputMapOnError** — the map is saved even when the loop ends in Error. *Why:*
  this contract was deliberately pinned early (partial maps must be persisted for
  scoring); a save-only-on-happy-path mutation dies exactly here.

### MapsComparison.* (13) — scoring + the exe
Basics/property: identical→100, fully-inverted→0, one-voxel-diff → exactly 124/125,
partially-mapped → exactly 56/64=87.5 (hand-counted walls/empties/unmapped),
Unmapped-over-Empty still agrees (the EX2 "unmapped=not-wall" rule), origin-smaller
scores over the overlap (100 then exactly 26/27), disjoint boxes → negative sentinel,
zero-sized grid → negative.

- **PartialOverlapScoresOverOverlapOnly** — origin [0,30)³ vs target [10,40)³, one
  disagreeing cell in the 8-cell overlap → exactly 7/8 = 87.5. *Why:* the
  "count cells outside the overlap" mutation was caught only by the disjoint-boxes
  sentinel test (a single fragile catcher); this adds an exact-value second carrier
  (mutated value would be 26/27).
- **NonzeroMinBoundsCompareCorrectly** — boxes at [100,200)×[100,200)×[300,430)
  (z-range deliberately mirrors the real house configs): identical→exactly 100, one
  flipped voxel → exactly 1299/1300, plus a different-offsets-but-world-aligned
  variant → 100. *Why:* a bounds-local-vs-world frame confusion in the target lookup
  survived EVERYTHING — every fixture's bounds started at zero, where local == world.
  The real staff configs start at z=300, so this bug would have silently mis-scored
  actual grading data.
- **PotentiallyOccupiedScoresAsNotWall** — asymmetric counts: 4 PO-over-Occupied
  (wrong) + 2 PO-over-Empty (correct) → exactly 60/64 = 93.75. *Why:* the documented
  "PO = not-wall" rule had no test with PO in a target map; and the counts MUST be
  asymmetric — with equal counts, flipping the rule swaps which cells are wrong but
  produces the same score (a subtle trap we computed before writing).
- **ExePrintsBareScoreOnStdout / ExeFailurePrintsMinusOneOnStdout** — spawn the real
  binary; parse the ENTIRE stdout as one number == 100 (any "score: " prefix or
  trailing text fails), exit 0; and: bad path → stdout exactly `-1`, non-empty stderr,
  nonzero exit. *Why:* these are the PDF's own acceptance checks for the exe and had
  ZERO coverage — both exe mutations survived trivially because no test ever executed
  the program.

### SimulationManager.* (9) — batch orchestration (weakest baseline: 8/15)
Basics: the PDF's grouped composition expands to exactly 20 runs ((2+3) missions × 2
drones × 2 lidars — the 2-vs-3 asymmetry is deliberate so grouping bugs change the
COUNT); one bad run scores −1 and the batch continues; a whole-group create-failure
scores −1 for exactly that group's 4 runs; the yaml carries score + resolution_status;
CLI path resolution (missing→`simulation.yaml`, relative stays relative, absolute
as-is).

- **ReportCarriesExactValuesSummaryAndFailureDetail** — three scripted runs
  {90 @ 5 steps, 80 @ 7 steps, create-throws → −1}: `score_range == (80, 90)` (the
  convention EXCLUDES −1 runs), the yaml's per-run `steps` re-read exactly (5 and 7),
  and the failed run still carries `RUN_CREATE_FAILED` detail in report AND yaml.
  *Why:* three separate survivors shared one root — the manager WROTE these values but
  no test ever READ them back (summary corruption, steps-written-as-score, and detail
  loss were all invisible).
- **ErrorsTxtImmediateWithCodeAndMessage** — run 0's create() throws; run 1's create()
  reads errors.txt *mid-batch* and must already see code + the distinctive message.
  *Why:* the PDF says errors are logged immediately, not deferred — but any post-run
  read can't distinguish the two (the stream flushes at close). The mock factory
  doubles as an in-flight observer; this also catches detail-less log lines.
- **PerRunOutputDirsAreDistinct** — captures the output paths handed to create():
  `run_0`, `run_1`, distinct, existing. *Why:* an index-collision mutation (every run
  writing to run_0, silently overwriting maps) was component-invisible — the mocked
  factory means the *path argument* is the manager's observable.
- **ParserReadsFieldsFromCorrectKeys** — writes a temp yaml tree where EVERY value is
  distinct (rotate 11 / advance 22 / elevate 33 / diameter 10→radius 5 / lidar
  6-300-2-3), parses, asserts each field. *Why:* a crossed yaml key (max_advance read
  from max_elevate_cm) survived everything — integration configs happened to have
  values where the swap didn't change behavior. Distinct-everything makes coincidence
  impossible; also pins the diameter→radius halving.

### Integration.* (20) + Radical.* (12)
- **RealStack tests**: full pipeline (parser → manager → factory → real everything) on
  tiny written-to-temp maps/configs — a scanned wall lands Occupied in the saved output
  map and scores; a multi-run composition aggregates and writes per-run outputs; a bad
  map file scores −1 and the batch continues; a crashing algorithm scores −1 and logs.
- **MockAlgorithm tests**: a scripted (deterministic) algorithm through the real stack —
  separates orchestration correctness from exploration behavior.
- **BenchmarkMap tests** (11): the real staff benchmark map (29×30×31) under varied
  bounds/starts/drone sizes/lidars, with score+steps thresholds. These are the tests
  that caught heading/altitude/frame mutations that component fixtures were too
  symmetric to see — and they're the efficiency measurement rig.
- **Radical tests** (12): adversarial configs — inverted/disjoint/oversized/negative
  bounds, zero fov, dense fov, zero gps resolution, negative map offset, and
  **GpsExceedsMapResIsGuardedAndClamped** (the guard: clamp + logged error + the run
  still completes).

## 6. Numbers to remember

- **126 tests / 12 suites / ~82s**, one binary, builds with `-j1` (machine constraint).
- ~105 single-line mutations measured; all 8 components at 100% of catchable ones;
  integration catch-rates 33–57% (bar was 20%); zero cross-suite failures.
- Full benchmark-map exploration: 34s wall-clock, score 100 (after the 2× scan-angle
  perf fix; 125s before).
- Two product-code hardening changes came OUT of testing: non-finite-pose guard in
  MockMovement and NaN-rejecting bounds in Map3DImpl — because a NaN-position mutation
  *segfaulted the whole test binary* (NaN passes both `< 0` and `>= n` checks, then UB
  on the size_t cast). Crash-class bugs now fail by assertion instead of killing the run.

## 7. Questions I'd expect, with the crisp answer

- *"Why fakes for some suites and real dependencies for others?"* — If the dependency is
  the suite's subject matter (converter writes into maps, comparison reads maps), a fake
  tests nothing. If it's scaffolding (lidar needs "a map", control needs "a pose
  source"), a real dependency makes the suite fail for OTHER components' bugs. We drew
  that line per-suite and documented it in readme.txt.
- *"How do you know your tests are good?"* — We measured them: injected ~105 realistic
  single-line bugs and required every one to turn a test red; every survivor got a
  targeted test that was verified red against that exact bug before being accepted.
- *"Give an example of a bug your tests originally missed."* — Best three: quitting at
  the first dead-end instead of backtracking (suite even ran faster and stayed green);
  the movement collision radius zeroed (all wall tests drove the centerline in); the
  score comparison reading the target map in bounds-local instead of world coordinates
  (all fixtures started at zero — the real house configs start at z=300).
- *"Anything your tests can't catch?"* — Yes, four documented equivalents (see §3
  ledger), each with a reason it's unobservable or uncontracted rather than untested.
- *"Why is a failed move fatal instead of replanning?"* — Design decision: the algorithm
  owns collision avoidance via the clearance margin (radius + gps/2 absorbs rounding);
  if ground truth still rejects a move, the belief model has been violated and the run
  is unsound → Error, −1. We prototyped the replan alternative and deliberately
  reverted it.
