# vdbscan-kitti-bench

Benchmarking crate for `vdbscan`. Loads local [KITTI](https://www.cvlibs.net/datasets/kitti/) Velodyne `.bin` scans and compares three DBSCAN backends head-to-head using [Criterion](https://github.com/bheisler/criterion.rs).

## Methods compared

| Method | Description |
| --- | --- |
| `vdbscan` | Morton-order voxel index — this library |
| `kiddo` | k-d tree neighbor search (bucket size 2048) |
| `bruteforce` | Naïve O(N²) pairwise distance search |

I/O (scan discovery, file read, and parsing) happens before each benchmark iteration. Only the clustering call is timed.

## Requirements

- A local copy of KITTI Velodyne point clouds (`.bin` files from the raw data or object detection split). The dataset is not bundled; download it from the [KITTI Vision Benchmark Suite](https://www.cvlibs.net/datasets/kitti/).
- `mise install` (installs `cargo-criterion` and the Rust toolchain).

## Running

### Via task (recommended)

```bash
# Single scan
task bench:kitti KITTI_PATH=/data/kitti/velodyne/000000.bin

# Directory of scans
task bench:kitti KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5
```

`KITTI_EPSILON` defaults to `0.4`, `KITTI_MIN_PTS` to `5`.

### Directly with cargo

```bash
KITTI_PATH=/data/kitti/velodyne \
KITTI_EPSILON=0.4 \
KITTI_MIN_PTS=5 \
cargo +1.89.0-x86_64-unknown-linux-gnu bench --locked -p vdbscan-kitti-bench --bench kitti
```

## Environment variables

| Variable | Default | Description |
| --- | --- | --- |
| `KITTI_PATH` | *(required)* | Path to a `.bin` file or a directory of `.bin` files |
| `KITTI_EPSILON` | `0.4` | DBSCAN epsilon radius in metres |
| `KITTI_MIN_PTS` | `5` | DBSCAN minimum neighbor count for core point |
| `KITTI_METHOD` | `vdbscan` | Method for the profiling binary (`vdbscan`, `kiddo`, `bruteforce`) |
| `KITTI_PROFILE_REPEATS` | `10` | Number of clustering iterations in the profiling binary |

`KITTI_METHOD` affects only the `perf` profiling binary, not the Criterion benchmark (which always runs all three methods).

## Output

### Criterion

Criterion writes results to `target/criterion/`. Open `target/criterion/report/index.html` in a browser for the full HTML report.

Each (method, scan) pair is a separate benchmark group. On a directory input, groups are named `vdbscan/000000.bin`, `kiddo/000000.bin`, etc., making it easy to compare methods on the same scan side by side. Throughput is reported in elements/second (points per second).

### Profiling with perf

Two tasks target the dedicated profiling binary instead of the Criterion harness:

```bash
# Release binary — trust these numbers for real hotspot analysis
task perf:release KITTI_PATH=/data/kitti/velodyne KITTI_METHOD=vdbscan

# Debug binary — easier source-level annotation, slower numbers
task perf:debug KITTI_PATH=/data/kitti/velodyne KITTI_METHOD=bruteforce
```

Output files are written to `reports/perf/<method>/` and named by short commit SHA:

```
reports/perf/vdbscan/kitti-release-a1b2c3d.perf.data
reports/perf/vdbscan/kitti-release-a1b2c3d.report.txt
reports/perf/vdbscan/kitti-release-a1b2c3d.annotate.txt
```

## Crate structure

```
benchmarks/kitti/
  src/
    lib.rs            — scan loading, three clustering implementations, config helpers
    bin/
      profile_kitti.rs — standalone profiling binary (perf target)
  benches/
    kitti.rs          — Criterion benchmark harness
  Cargo.toml
```

`lib.rs` provides `load_kitti_scan`, `benchmark_config_from_env`, and the `BenchmarkMethod` enum used by both the Criterion harness and the profiling binary.
