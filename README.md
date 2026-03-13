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
- [ ] Input codecs (PCD, PLY, LAS, CSV)
- [ ] Output / visualization

## References

- Ester et al. 1996 — *A Density-Based Algorithm for Discovering Clusters*, KDD 1996
- Niessner et al. 2013 — *Real-time 3D Reconstruction at Scale using Voxel Hashing*, SIGGRAPH Asia
