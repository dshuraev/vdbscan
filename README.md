# vdbscan

> **Work in progress** — core algorithm complete, I/O codecs and visualization not yet implemented.

DBSCAN point-cloud clustering in Rust, accelerated by a voxel hash index.

## Algorithm

[DBSCAN](https://en.wikipedia.org/wiki/DBSCAN) groups points by density without requiring a predetermined cluster count. Noise points (insufficient neighbors) are returned as `None`; cluster members carry a 1-based `NonZeroUsize` id.

Neighbor lookup uses a **voxel hash index** — space is discretized into cells of side length `epsilon`. A neighborhood query checks the 27-cell (3×3×3) cube around a point, giving O(1) amortized lookup and O(N) index construction.

| Method        | Complexity                                                                        |
| naive DBSCAN  | O(N²) distance comparisons                                                        |
| `vdbscan`     | O(N) index build  +  O(N·k) labeling   (k = avg candidates per 27-cell window)    |

## Usage

```rust
use vdbscan::{dbscan, Point3};

let points: Vec<Point3> = /* load your scan */;

// epsilon  — neighborhood radius in the same units as your coordinates (e.g. 0.4 m)
// min_pts  — minimum neighbors for a point to be considered a core point (e.g. 5)
let labels = dbscan(&points, 0.4, 5);

for (i, label) in labels.iter().enumerate() {
    match label {
        Some(id) => println!("point {i} → cluster {id}"),
        None     => println!("point {i} → noise"),
    }
}
```

## Parameters

| Parameter | Type    | Meaning                                                                 |
|-----------|---------|-------------------------------------------------------------------------|
| `epsilon` | `f32`   | Neighborhood radius. Maps to physical spacing — e.g. 0.3–0.5 m for LiDAR vehicle returns. |
| `min_pts` | `usize` | Minimum neighbors (inclusive) to classify a point as a core point. Typically 5–10. |

## Status

- [x] `Point3` / `VoxelKey` / `ClusterLabel` types
- [x] `VoxelIndex` — O(N) build, O(1) 27-cell neighborhood query
- [x] `dbscan` — core-point detection + BFS cluster expansion
- [x] External KITTI benchmark crate for local Velodyne `.bin` scans
- [ ] Input codecs (PCD, PLY, LAS, CSV)
- [ ] Output / visualization

## Benchmarking

KITTI benchmarking lives outside the library crate in `benchmarks/kitti`. The benchmark crate loads local KITTI Velodyne `.bin` scans, parses `(x, y, z, intensity)` records, drops intensity, and times three clustering backends with Criterion:

- `vdbscan` - voxel-hash index
- `kiddo` - k-d tree neighbor search
- `bruteforce` - naive O(N^2) neighbor search

The timed section benchmarks clustering only. Scan discovery, file I/O, and parsing happen before each benchmark iteration.

Run it against a single scan file or a directory of scans:

```bash
VDBSCAN_KITTI_PATH=/data/kitti/velodyne/000000.bin \
cargo +1.89.0-x86_64-unknown-linux-gnu bench --locked -p vdbscan-kitti-bench --bench kitti
```

Or use the task wrapper:

```bash
task bench:kitti KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5
```

The benchmark also accepts environment variables:

```bash
VDBSCAN_KITTI_PATH=/data/kitti/velodyne \
VDBSCAN_KITTI_EPSILON=0.4 \
VDBSCAN_KITTI_MIN_PTS=5 \
cargo +1.89.0-x86_64-unknown-linux-gnu bench --locked -p vdbscan-kitti-bench --bench kitti
```

Criterion emits a separate trace for each method and scan pair, so the HTML report shows side-by-side series such as `vdbscan/000000.bin`, `kiddo/000000.bin`, and `bruteforce/000000.bin`.

For profiling, generate a flamegraph from the same KITTI input:

```bash
task perf:flamegraph KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5 KITTI_METHOD=kiddo
```

The flamegraph is written to `reports/perf/kitti.svg`.

For text-first profiling and hot-line inspection, use `perf` against the dedicated profiling binary instead of the Criterion harness:

```bash
task perf:release KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5 KITTI_METHOD=vdbscan
task perf:debug KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5 KITTI_METHOD=bruteforce
```

Outputs:

- `reports/perf/kitti-release.perf.data` and `reports/perf/kitti-release.annotate.txt`
- `reports/perf/kitti-debug.perf.data` and `reports/perf/kitti-debug.annotate.txt`

`perf:release` is the one to trust for real hotspots. `perf:debug` is mainly for easier source-level inspection.

`profile_kitti` accepts `VDBSCAN_KITTI_METHOD` with `vdbscan`, `kiddo`, or `bruteforce`. If unset, profiling defaults to `vdbscan`.

## References

- Ester et al. 1996 — *A Density-Based Algorithm for Discovering Clusters*, KDD 1996
- Niessner et al. 2013 — *Real-time 3D Reconstruction at Scale using Voxel Hashing*, SIGGRAPH Asia
