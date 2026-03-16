# vdbscan

DBSCAN point-cloud clustering in Rust, accelerated by a Morton-order voxel index.

## Quickstart

### Required software

| Tool                                                              | Purpose                                               |
| ----------------------------------------------------------------- | ----------------------------------------------------- |
| [mise](https://mise.jdx.dev/)                                     | Manages the Rust toolchain and all cargo tools        |
| [Docker](https://www.docker.com/) or [Podman](https://podman.io/) | `task test:cross` only — cross-compilation to aarch64 |
| `perf` (Linux)                                                    | `task perf:release` / `task perf:debug` only          |

### Setup

```bash
mise install          # install Rust 1.89.0, nightly, and all cargo tools
task                  # format check → clippy → debug build
```

### Run the CLI

```bash
task build:cli:release

# cluster a KITTI scan, write labelled PLY
vdbscan -i scan.bin -o clusters.ply --epsilon 0.4 --min-pts 5

# cluster a CSV, write CSV output
vdbscan -i cloud.csv -o labels.csv --epsilon 0.5 --min-pts 3

# override format detection
vdbscan -i scan -o out.ply --epsilon 0.4 --min-pts 5 --input-format kitti
```

See [vdbscan-cli/README.md](vdbscan-cli/README.md) for the full CLI reference.

## Library API

```rust
use vdbscan::{dbscan, PointCloud};

let mut cloud = PointCloud::with_capacity(points.len());
for (x, y, z) in points {
    cloud.push(x, y, z);
}

// epsilon  — neighborhood radius (same units as coordinates, e.g. 0.4 m for LiDAR)
// min_pts  — minimum neighbors for a point to qualify as a core point (e.g. 5)
let clustering = dbscan(cloud, 0.4, 5);

for (point, label) in clustering.iter() {
    match label {
        Some(id) => println!("({:.2},{:.2},{:.2}) → cluster {id}", point.x, point.y, point.z),
        None     => println!("({:.2},{:.2},{:.2}) → noise",        point.x, point.y, point.z),
    }
}

println!("{} clusters, {} noise points", clustering.cluster_count(), clustering.noise_count());
```

### Parameters

| Parameter | Type    | Meaning                                                                                 |
| --------- | ------- | --------------------------------------------------------------------------------------- |
| `epsilon` | `f32`   | Neighborhood radius. For vehicle LiDAR returns, 0.3–0.5 m is a typical starting range.  |
| `min_pts` | `usize` | Minimum neighbor count (inclusive) to classify a point as a core point. Typically 5–10. |

### Output types

- **`ClusterLabel`** — `Option<NonZeroUsize>`: `None` means noise; `Some(id)` is a 1-based cluster id.
- **`Clustering`** — the clustered point cloud and a parallel label vector. Points may be reordered relative to the input (Morton order), but `clustering.cloud[i]` and `clustering.labels[i]` always refer to the same point.

## Theory

### DBSCAN

[DBSCAN](https://en.wikipedia.org/wiki/DBSCAN) (Ester et al., 1996) discovers clusters by density without requiring a predetermined cluster count. Every point falls into one of three roles:

- **Core point** — has ≥ `min_pts` neighbors within radius ε (the point itself is not counted).
- **Border point** — within ε of a core point but has too few neighbors of its own.
- **Noise** — neither core nor border.

Clusters are maximal connected sets of core points (connected = within ε of each other), extended to include any border points reachable from them. Noise points are returned with label `None`.

Two parameters control the shape of clusters:

- Increasing **ε** merges nearby clusters and absorbs more noise.
- Increasing **min_pts** requires denser regions to form a cluster, suppressing small groups.

Unlike k-means, DBSCAN handles clusters of arbitrary shape and naturally identifies outliers.

### Voxel hash acceleration

The naïve approach compares every point pair — O(N²) distance evaluations. This hard bound converts neighbor lookup from O(N) to O(1) amortized:

| Method       | Complexity                                                                     |
| ------------ | ------------------------------------------------------------------------------ |
| Naïve DBSCAN | O(N²) distance comparisons                                                     |
| `vdbscan`    | O(N) index build + O(N·k) labeling, k = average candidates per 27-cell window  |

`vdbscan` always searches a fixed `3×3×3` voxel neighborhood (27 cells). To keep that search complete for `ε`-radius queries, the voxel size is chosen as the next representable `f32` value above `ε`.

### Morton (Z-order) encoding

Voxel lookup efficiency depends on how voxels are stored. A naive hash map incurs random memory access and cache misses. `vdbscan` instead sorts all points by their [Morton (Z-order) code](https://en.wikipedia.org/wiki/Z-order_curve):

1. Each voxel's integer coordinates (vx, vy, vz) are encoded into a single 63-bit key by interleaving the bits of all three axes.
2. Points are sorted by Morton key, so spatially close voxels land close together in the sorted array — exploiting cache lines for neighbor traversal.
3. Consecutive runs of the same key are compressed into **voxel spans** (start index + length).
4. For each span, the 27 neighboring spans are precomputed once at index-build time. Candidate iteration during clustering never searches — it just walks a precomputed list of span pointers.

Binary search over the sorted span array finds any voxel in O(log V) where V is the number of occupied voxels. Because the index is built O(N) overall (sort dominates), the total clustering cost is O(N log N) in the worst case and O(N) in practice for typical point clouds where V ≪ N.

## Algorithm

Two passes over the spatial index:

**Pass 1 — core-point classification.** For each point, count neighbors within radius ε by iterating over candidate spans. If the count reaches `min_pts`, the point is flagged as a core point.

**Pass 2 — BFS cluster expansion.** Iterate over all unvisited core points. When an unlabelled core point is found, start a breadth-first search: assign a new cluster id to the point, enqueue its ε-neighbors that are also core points, and continue until the queue is empty. Border points (within ε of any core point but not core themselves) receive the id of the first core point that reaches them. All remaining unvisited points are noise.

BFS is used instead of recursion to avoid stack overflow on large, elongated clusters.

## Tasks

Run `task --list` to see all available tasks. Common ones:

| Task                             | Description                                                          |
| -------------------------------- | -------------------------------------------------------------------- |
| `task`                           | Format check → clippy → debug build (default fast loop)              |
| `task build`                     | Debug build — whole workspace                                        |
| `task build:cli`                 | Debug build — CLI binary only                                        |
| `task build:cli:release`         | Release build — CLI binary only                                      |
| `task build:release`             | Release build — whole workspace                                      |
| `task test`                      | Run all tests (native)                                               |
| `task test:cross`                | Run tests on aarch64 via cross *(requires Docker/Podman)*            |
| `task clippy`                    | Clippy with `-D warnings`                                            |
| `task format`                    | `rustfmt --check`                                                    |
| `task coverage`                  | LLVM coverage report (text + lcov + HTML)                            |
| `task miri`                      | UB detection under Miri (nightly)                                    |
| `task fuzz`                      | Fuzz all targets with a fixed run budget                             |
| `task audit`                     | Audit crates for known CVEs                                          |
| `task deny`                      | Supply-chain checks (licenses, bans, sources)                        |
| `task ci`                        | Full CI gate: format → clippy → test → cross-test → deny → build all |
| `task check`                     | Everything including miri, fuzz, coverage, bloat, coupling           |
| `task bench:kitti KITTI_PATH=…`  | Criterion benchmark against local KITTI scans                        |
| `task perf:release KITTI_PATH=…` | `perf record` + report + annotate (release binary)                   |
| `task perf:debug KITTI_PATH=…`   | Same with debug binary for source-level inspection                   |

## Benchmarking

The benchmark crate at `benchmarks/kitti` loads local KITTI Velodyne `.bin` scans, drops the intensity channel, and times three clustering backends with [Criterion](https://github.com/bheisler/criterion.rs):

- **vdbscan** — Morton-order voxel index (this library)
- **kiddo** — k-d tree neighbor search
- **bruteforce** — naïve O(N²) pairwise search

I/O and parsing happen outside the timed section. See [benchmarks/kitti/README.md](benchmarks/kitti/README.md) for details.

```bash
task bench:kitti KITTI_PATH=/data/kitti/velodyne KITTI_EPSILON=0.4 KITTI_MIN_PTS=5
```

Criterion writes an HTML report under `target/criterion/`. Each (method, scan) pair is a separate series, e.g. `vdbscan/000000.bin`, `kiddo/000000.bin`, `bruteforce/000000.bin`.

For hot-line profiling with `perf`:

```bash
task perf:release KITTI_PATH=/data/kitti/velodyne KITTI_METHOD=vdbscan
task perf:debug   KITTI_PATH=/data/kitti/velodyne KITTI_METHOD=bruteforce
```

Reports are written to `reports/perf/<method>/kitti-<profile>-<sha>.(perf.data|report.txt|annotate.txt)`.

## Future work

- **Parallel index build.** The sort and span-compression passes in `MortonIndex::build` are single-threaded. Radix sort and parallel prefix-sum are natural fits.
- **GPU acceleration.** Morton encoding and the core-point counting pass are embarrassingly parallel and map cleanly to compute shaders or CUDA kernels.
- **Adaptive voxel size.** At very low point densities even the 27-cell window can waste work on empty voxels. A multi-resolution index (octree-style) could skip empty levels.
- **Streaming / incremental updates.** Re-inserting a batch of new points without rebuilding the full index from scratch.
- **Additional I/O formats.** PCD (ROS), LAS/LAZ (aerial LiDAR), E57.
- **Python bindings.** PyO3 wrapper exposing `dbscan(numpy_xyz, epsilon, min_pts) -> numpy_labels`.

## References

- Ester et al. 1996 — *A Density-Based Algorithm for Discovering Clusters*, KDD 1996
- Niessner et al. 2013 — *Real-time 3D Reconstruction at Scale using Voxel Hashing*, SIGGRAPH Asia
- Morton 1966 — *A computer oriented geodetic data base and a new technique in file sequencing*, IBM Technical Report
