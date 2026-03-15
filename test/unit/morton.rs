use std::collections::BTreeSet;

use vdbscan::PointCloud;
use vdbscan::morton::{
    MORTON_COORD_MAX, MORTON_COORD_MIN, MortonIndex, SpanLookup, VoxelSpan, morton_encode_batch,
    morton_encode_voxel, voxelize_batch,
};

// ---------------------------------------------------------------------------
// Morton encoding
// ---------------------------------------------------------------------------

/// The minimum representable voxel coordinate (= -MORTON_BIAS = -2^20).
/// It must map to Morton key 0 (all input bits zero after bias removal).
#[test]
fn test_morton_min_coord_maps_to_zero() {
    const MIN: i32 = -(1 << 20);
    assert_eq!(morton_encode_voxel(MIN, MIN, MIN), 0);
}

/// Each axis contributes to a distinct set of bits, so the all-bias point
/// (0, 0, 0 in voxel space) produces a known non-zero key.
/// The bias is 2^20, so each axis spreads bit 20 → bit 60.
/// Key = 2^60 | (2^60 << 1) | (2^60 << 2) = 0x7000_0000_0000_0000.
#[test]
fn test_morton_origin_known_value() {
    assert_eq!(morton_encode_voxel(0, 0, 0), 0x7000_0000_0000_0000);
}

/// Moving +1 in x (with y, z constant) must strictly increase the key,
/// because x contributes to bits 0, 3, 6, … and y/z are unchanged.
#[test]
fn test_morton_x_axis_strictly_increasing() {
    let k0 = morton_encode_voxel(0, 0, 0);
    let k1 = morton_encode_voxel(1, 0, 0);
    let k2 = morton_encode_voxel(2, 0, 0);
    assert!(k0 < k1, "k0={k0} k1={k1}");
    assert!(k1 < k2, "k1={k1} k2={k2}");
}

#[test]
fn test_morton_distinct_axes_produce_distinct_keys() {
    let kx = morton_encode_voxel(1, 0, 0);
    let ky = morton_encode_voxel(0, 1, 0);
    let kz = morton_encode_voxel(0, 0, 1);
    assert_ne!(kx, ky);
    assert_ne!(kx, kz);
    assert_ne!(ky, kz);
}

#[test]
fn test_morton_negative_coords_distinct() {
    let kneg = morton_encode_voxel(-1, -1, -1);
    let kzero = morton_encode_voxel(0, 0, 0);
    let kpos = morton_encode_voxel(1, 1, 1);
    assert!(kneg < kzero);
    assert!(kzero < kpos);
}

#[test]
fn test_morton_deterministic() {
    let a = morton_encode_voxel(7, -3, 15);
    let b = morton_encode_voxel(7, -3, 15);
    assert_eq!(a, b);
}

/// `morton_encode_batch` must match element-wise calls to `morton_encode_voxel`.
#[test]
fn test_morton_encode_batch_matches_scalar() {
    let vx = [0i32, 1, -1, 5, -100];
    let vy = [0i32, 0, 1, -2, 50];
    let vz = [0i32, -1, 0, 3, 0];
    let batch = morton_encode_batch(&vx, &vy, &vz);
    for i in 0..vx.len() {
        assert_eq!(
            batch[i],
            morton_encode_voxel(vx[i], vy[i], vz[i]),
            "mismatch at index {i}"
        );
    }
}

/// Extreme voxel coordinates (i32::MAX / i32::MIN) must not alias in-range
/// coordinates after the 21-bit Morton mask.  Without clamping to
/// MORTON_COORD_MIN/MAX, i32::MAX and -1 produce the same key.
#[test]
fn test_morton_extreme_coords_do_not_alias() {
    // i32::MAX would alias with -1 without clamping; clamp maps it to MORTON_COORD_MAX.
    assert_ne!(
        morton_encode_voxel(i32::MAX, i32::MAX, i32::MAX),
        morton_encode_voxel(-1, -1, -1),
        "i32::MAX must not alias -1 after clamping"
    );
    // i32::MIN would alias with 0 without clamping; clamp maps it to MORTON_COORD_MIN.
    assert_ne!(
        morton_encode_voxel(i32::MIN, i32::MIN, i32::MIN),
        morton_encode_voxel(0, 0, 0),
        "i32::MIN must not alias 0 after clamping"
    );
    // Clamped extremes must equal the boundary values.
    assert_eq!(
        morton_encode_voxel(i32::MAX, i32::MAX, i32::MAX),
        morton_encode_voxel(MORTON_COORD_MAX, MORTON_COORD_MAX, MORTON_COORD_MAX),
    );
    assert_eq!(
        morton_encode_voxel(i32::MIN, i32::MIN, i32::MIN),
        morton_encode_voxel(MORTON_COORD_MIN, MORTON_COORD_MIN, MORTON_COORD_MIN),
    );
}

// ---------------------------------------------------------------------------
// Voxelization
// ---------------------------------------------------------------------------

#[test]
fn test_voxelize_batch_floor_positive() {
    let px = [0.0f32, 0.49, 0.5, 1.99];
    let py = [0.0f32; 4];
    let pz = [0.0f32; 4];
    let mut vx = [0i32; 4];
    let mut vy = [0i32; 4];
    let mut vz = [0i32; 4];
    voxelize_batch(&px, &py, &pz, 1.0 / 0.5, &mut vx, &mut vy, &mut vz);
    assert_eq!(vx, [0, 0, 1, 3]);
}

#[test]
fn test_voxelize_batch_floor_negative() {
    let px = [-0.1f32, -0.5, -1.0];
    let py = [0.0f32; 3];
    let pz = [0.0f32; 3];
    let mut vx = [0i32; 3];
    let mut vy = [0i32; 3];
    let mut vz = [0i32; 3];
    voxelize_batch(&px, &py, &pz, 1.0 / 0.5, &mut vx, &mut vy, &mut vz);
    // floor(-0.1/0.5) = floor(-0.2) = -1
    // floor(-0.5/0.5) = floor(-1.0) = -1
    // floor(-1.0/0.5) = floor(-2.0) = -2
    assert_eq!(vx, [-1, -1, -2]);
}

// ---------------------------------------------------------------------------
// MortonIndex build
// ---------------------------------------------------------------------------

#[test]
fn test_build_empty_cloud() {
    let index = MortonIndex::build(PointCloud::new(), 1.0);
    assert!(index.sorted_cloud.is_empty());
    assert!(index.voxel_spans.is_empty());
    assert!(index.point_span_lut.is_empty());
    assert!(index.neighbors.is_empty());
}

#[test]
fn test_build_single_point() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5);
    let index = MortonIndex::build(cloud, 1.0);

    assert_eq!(index.sorted_cloud.len(), 1);
    assert_eq!(index.voxel_spans.len(), 1);
    assert_eq!(index.voxel_spans[0].start, 0);
    assert_eq!(index.voxel_spans[0].len, 1);
    assert_eq!(index.point_span_lut[0], 0);
    assert_eq!(index.neighbors.len(), 1);
}

/// Points in the same voxel must produce exactly one span.
#[test]
fn test_same_voxel_is_one_span() {
    let mut cloud = PointCloud::new();
    // All three sit in voxel (0,0,0) with eps=1.0.
    cloud.push(0.1, 0.1, 0.1);
    cloud.push(0.5, 0.5, 0.5);
    cloud.push(0.9, 0.9, 0.9);
    let index = MortonIndex::build(cloud, 1.0);

    assert_eq!(index.voxel_spans.len(), 1);
    assert_eq!(index.voxel_spans[0].len, 3);
}

/// Points in three separate voxels must produce three spans.
#[test]
fn test_separate_voxels_produce_separate_spans() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5); // voxel (0,0,0)
    cloud.push(1.5, 0.5, 0.5); // voxel (1,0,0)
    cloud.push(2.5, 0.5, 0.5); // voxel (2,0,0)
    let index = MortonIndex::build(cloud, 1.0);

    assert_eq!(index.voxel_spans.len(), 3);
    assert_eq!(index.sorted_cloud.len(), 3);
    // Spans must be sorted by morton_key.
    for w in index.voxel_spans.windows(2) {
        assert!(w[0].morton_key < w[1].morton_key);
    }
}

/// The total points across all spans must equal the input cloud size.
#[test]
fn test_span_lengths_sum_to_n() {
    let mut cloud = PointCloud::new();
    for blob_x in [0.0f32, 5.0, 10.0] {
        for i in 0..7 {
            cloud.push(blob_x + i as f32 * 0.1, 0.0, 0.0);
        }
    }
    let n = cloud.len();
    let index = MortonIndex::build(cloud, 1.0);

    let total: usize = index.voxel_spans.iter().map(|s| s.len).sum();
    assert_eq!(total, n);
}

/// Every sorted point must map to a span that actually contains it.
#[test]
fn test_point_span_lut_is_consistent() {
    let mut cloud = PointCloud::new();
    for i in 0..20 {
        cloud.push(i as f32 * 0.3, (i % 3) as f32, 0.0);
    }
    let index = MortonIndex::build(cloud, 1.0);

    for i in 0..index.sorted_cloud.len() {
        let span_idx = index.point_span_lut[i];
        let span = &index.voxel_spans[span_idx];
        assert!(
            i >= span.start && i < span.start + span.len,
            "point {i} is not contained in its claimed span {span:?}"
        );
    }
}

/// The sorted cloud must contain the same set of points as the input
/// (coordinates as bit patterns, order-independent).
#[test]
fn test_sorted_cloud_contains_all_input_points() {
    let mut cloud = PointCloud::new();
    let coords = [(1.1f32, 2.2, 3.3), (-0.5, 0.0, 4.0), (7.0, -1.0, 0.5)];
    for &(x, y, z) in &coords {
        cloud.push(x, y, z);
    }

    let original: BTreeSet<(u32, u32, u32)> = coords
        .iter()
        .map(|&(x, y, z)| (x.to_bits(), y.to_bits(), z.to_bits()))
        .collect();

    let index = MortonIndex::build(cloud, 1.0);
    let sorted: BTreeSet<(u32, u32, u32)> = (0..index.sorted_cloud.len())
        .map(|i| {
            let p = index.sorted_cloud.get(i);
            (p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
        })
        .collect();

    assert_eq!(original, sorted);
}

// ---------------------------------------------------------------------------
// Neighbor precomputation
// ---------------------------------------------------------------------------

/// A span's own index must appear in its own neighbor list (self-inclusion).
#[test]
fn test_neighbors_include_self() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5);
    let index = MortonIndex::build(cloud, 1.0);

    assert!(
        index.neighbors[0].contains(&0),
        "span 0 must list itself as a neighbor"
    );
}

/// Two points in adjacent voxels must each find the other's span as a neighbor.
#[test]
fn test_adjacent_voxels_are_mutual_neighbors() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5); // voxel (0,0,0)
    cloud.push(1.5, 0.5, 0.5); // voxel (1,0,0)
    let index = MortonIndex::build(cloud, 1.0);

    assert_eq!(index.voxel_spans.len(), 2);

    // Find span indices for each voxel.
    let span_of = |x: f32| -> usize {
        let p = index.sorted_cloud.get(
            (0..index.sorted_cloud.len())
                .find(|&i| (index.sorted_cloud.vx[i] - x).abs() < 0.01)
                .unwrap(),
        );
        let _ = p;
        let point_idx = (0..index.sorted_cloud.len())
            .find(|&i| (index.sorted_cloud.vx[i] - x).abs() < 0.01)
            .unwrap();
        index.point_span_lut[point_idx]
    };

    let s0 = span_of(0.5);
    let s1 = span_of(1.5);

    assert!(
        index.neighbors[s0].contains(&s1),
        "voxel at x=0.5 must list voxel at x=1.5 as a neighbor"
    );
    assert!(
        index.neighbors[s1].contains(&s0),
        "voxel at x=1.5 must list voxel at x=0.5 as a neighbor"
    );
}

/// Spans that are farther than NEIGHBOR_RADIUS voxels apart must not appear
/// in each other's neighbor lists.
#[test]
fn test_distant_voxels_are_not_neighbors() {
    use vdbscan::NEIGHBOR_RADIUS;

    let gap = (NEIGHBOR_RADIUS + 2) as f32; // guaranteed beyond the radius
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5);
    cloud.push(0.5 + gap, 0.5, 0.5);
    let index = MortonIndex::build(cloud, 1.0);

    assert_eq!(index.voxel_spans.len(), 2);
    assert!(
        !index.neighbors[0].contains(&1),
        "spans {gap} voxels apart must not be neighbors"
    );
}

// ---------------------------------------------------------------------------
// candidates() iterator
// ---------------------------------------------------------------------------

/// candidates() for a point must include the point itself (self-span).
#[test]
fn test_candidates_include_self() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5);
    let index = MortonIndex::build(cloud, 1.0);

    let cands: Vec<usize> = index.candidates(0).collect();
    assert!(cands.contains(&0));
}

/// candidates() must yield every point in a neighboring span.
#[test]
fn test_candidates_span_adjacent_voxel() {
    let mut cloud = PointCloud::new();
    cloud.push(0.5, 0.5, 0.5); // voxel A
    cloud.push(1.5, 0.5, 0.5); // voxel B (adjacent)
    let index = MortonIndex::build(cloud, 1.0);

    // Find which sorted index holds the point at x=0.5.
    let idx_a = (0..2)
        .find(|&i| (index.sorted_cloud.vx[i] - 0.5).abs() < 0.01)
        .unwrap();
    let idx_b = 1 - idx_a;

    let cands_a: Vec<usize> = index.candidates(idx_a).collect();
    let cands_b: Vec<usize> = index.candidates(idx_b).collect();

    assert!(
        cands_a.contains(&idx_b),
        "point A must find point B as candidate"
    );
    assert!(
        cands_b.contains(&idx_a),
        "point B must find point A as candidate"
    );
}

/// Every candidate index must be a valid index into sorted_cloud.
#[test]
fn test_candidates_indices_are_in_bounds() {
    let mut cloud = PointCloud::new();
    for i in 0..15 {
        cloud.push(i as f32 * 0.4, (i % 4) as f32 * 0.4, 0.0);
    }
    let n = cloud.len();
    let index = MortonIndex::build(cloud, 1.0);

    for i in 0..n {
        for c in index.candidates(i) {
            assert!(c < n, "candidate {c} out of bounds (n={n})");
        }
    }
}

// ---------------------------------------------------------------------------
// Custom SpanLookup (swappability smoke test)
// ---------------------------------------------------------------------------

/// A linear-scan lookup that wraps a slice reference.
struct LinearScanLookup;

impl SpanLookup for LinearScanLookup {
    fn find(&self, spans: &[VoxelSpan], key: u64) -> Option<usize> {
        spans.iter().position(|s| s.morton_key == key)
    }
}

/// Building with a custom lookup must produce the same neighbor lists as the
/// default binary-search strategy.
#[test]
fn test_custom_lookup_matches_binary_search() {
    let mut cloud = PointCloud::new();
    for i in 0..10 {
        cloud.push(i as f32 * 0.6, 0.0, 0.0);
    }

    let cloud2 = cloud.clone();
    let default_idx = MortonIndex::build(cloud, 1.0);
    let custom_idx = MortonIndex::build_with_lookup(cloud2, 1.0, &LinearScanLookup);

    assert_eq!(default_idx.voxel_spans.len(), custom_idx.voxel_spans.len());
    for (a, b) in default_idx
        .neighbors
        .iter()
        .zip(custom_idx.neighbors.iter())
    {
        let mut sa: Vec<usize> = a.to_vec();
        let mut sb: Vec<usize> = b.to_vec();
        sa.sort_unstable();
        sb.sort_unstable();
        assert_eq!(sa, sb);
    }
}
