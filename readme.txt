drone_mapper - output formats
==============================

The simulator is run as:

    drone_mapper_simulation [<simulation.yaml>] [<output_path>]

      <simulation.yaml> : composition file. Missing -> "simulation.yaml" in the cwd.
                          A relative path is relative to the cwd; an absolute path is
                          used as-is.
      <output_path>     : where outputs are written. Missing -> the cwd.

It writes exactly two artifacts under <output_path>:
  1. simulation_output.yaml   (the aggregate score report)
  2. output_results/          (per-run output maps + the error log)

The standalone scorer is run as:

    maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<path>]

  It prints ONLY the score (0-100) to stdout. On any error (bad/missing file, or no
  overlapping region) it prints -1 to stdout and a description to stderr.


1) simulation_output.yaml
-------------------------
One YAML document. Hierarchy: report -> runs -> (per-run) missions.

  metric:            string  - the scoring metric ("cell_accuracy_percent")
  generated_at_utc:  string  - ISO-8601 UTC timestamp of the report
  error_score:       int     - the score given to a failed run (always -1)
  score_range:               - min/max over the runs that did NOT fail
    min: number              - (both -1 if every run failed)
    max: number
  runs:              list     - one entry per expanded run, in expansion order

  Each entry under `runs` (one simulation run):
    map_filename:       string  - the hidden map this run used
    score:              number  - cell accuracy in [0,100], or -1 if the run failed
    resolution_status:  string  - Accepted | Ignored | IgnoredTooSmall
                                  (the requested output_mapping_resolution_factor;
                                   the run always uses the default resolution)
    output_map_file:    string  - absolute path to this run's saved output map
    missions:           list    - one entry per mission executed in this run
      - status:  string         - Completed | MaxSteps | Error
        steps:   int            - number of drone steps taken
        errors:  list           - present only when the run logged errors
          - code:    string
            message: string

  Run expansion order (this also fixes the run_<index> numbering below):
    for each (simulation, missions) group in the composition:
      for each mission in that group:
        for each drone:
          for each lidar:
            -> one run


2) output_results/
------------------
A folder created under <output_path>:

  output_results/
    errors.txt          - every run error, written immediately as it occurs (not
                          batched). One line per error:
                              run <index>: [<code>] <message>
    run_0/
      output_map.npy    - the output occupancy map produced by run 0
    run_1/
      output_map.npy    - the output occupancy map produced by run 1
    ...                 - run_<index> for index 0..N-1, matching the expansion order

  output_map.npy format (TinyNPY / NumPy .npy):
    dtype      : uint8 ("|u1")
    order      : C-order, axis order [x][y][z]
    linear idx : x*ny*nz + y*nz + z
    encoding   : 0 = Empty
                 1 = Occupied
                 253 = PotentiallyOccupied
                 255 = Unmapped
                 (OutOfBounds is never stored; it is implied by the grid extent.)
    extent     : the grid covers the mission's mapping box; the map offset is the
                 box minimum, and the cell size is the output resolution.

Assumptions:
  - The planner assumes gps_resolution <= map_resolution; outside that range it is
    guarded: gps_resolution is clamped down to map_resolution for the run and a
    GPS_RES_EXCEEDS_MAP_RES note is written to output_results/errors.txt.

Testing note:
  - ScanResultToVoxels and MapsComparison component tests intentionally run against
    the real Map3DImpl: their subject matter is map interaction (writing scans into /
    scoring against real voxel storage), so a fake would test nothing. MockLidar,
    DroneControl and MappingAlgorithm component tests instead use a hand-rolled
    FakeMap3D fixture (tests/components/FakeMap3D.h) so defects in Map3DImpl cannot
    fail suites whose subject only incidentally needs a map.
  - Similarly, MockLidar and DroneControl component tests use a hand-rolled FakeGps
    fixture (tests/components/FakeGps.h) instead of the real MockGPS: the GPS there is
    only a pose source. SimulationRun keeps the real MockGPS/MockMovement -- it is the
    suite that grades them.
