//! Morton-key (Z-order curve) spatial index for voxel-accelerated DBSCAN.
//!
//! See `docs/dev/morton.md` for the full design specification.
//!
//! Build pipeline:
//!
//! ```text
//! PointCloud
//!    ↓ voxelize + Morton encode          (voxelize_batch, morton_encode_batch)
//! MortonRef[(key, source_index)]
//!    ↓ sort by key
//! MortonRef sorted
//!    ↓ gather
//! sorted_cloud
//!    ↓ run-length compress
//! voxel_spans
//!    ↓ fill point → span table
//! point_span_lut
//!    ↓ neighbor enumeration              (SpanLookup)
//! neighbors
//! ```
//!
//! The clustering phase then accesses only `sorted_cloud`, `voxel_spans`,
//! `point_span_lut`, and `neighbors`; all temporary build structures are
//! dropped automatically at the end of [`MortonIndex::build_with_lookup`].

use smallvec::SmallVec;

use crate::types::PointCloud;

// ---------------------------------------------------------------------------
// Compile-time neighbor radius
// ---------------------------------------------------------------------------

/// Neighborhood search radius in voxels.
///
/// The cube inspected per span during neighbor precomputation has side length
/// `2 * NEIGHBOR_RADIUS + 1`:
///
/// | Radius | Cube size |
/// |--------|-----------|
/// | 1      | 27        |
///
pub const NEIGHBOR_RADIUS: i32 = 1;

/// Inline capacity of [`NeighborList`].
///
/// Chosen to be at least the theoretical maximum (`cube_width³`) for the
/// configured radius so heap allocation is avoided for any valid neighbor
/// list.
///
/// | Radius | True max | Inline cap |
/// |--------|----------|------------|
/// | 1      | 27       | 27         |
const MAX_NEIGHBORS: usize = 27;

/// Neighbor span list for one voxel span.
///
/// `SmallVec` avoids heap allocation when the number of occupied neighbor
/// spans fits within `MAX_NEIGHBORS` (the theoretical maximum for the
/// configured radius). For typical sparse point clouds this inline path is
/// almost always taken.
///
/// To use a different container (e.g. `Vec<usize>` to reduce struct size for
/// very dense data), replace this alias and recompile.
pub type NeighborList = SmallVec<[usize; MAX_NEIGHBORS]>;

// ---------------------------------------------------------------------------
// Morton encoding
// ---------------------------------------------------------------------------

/// Bias that shifts i32 voxel coordinates from `[-2^20, 2^20)` to `[0, 2^21)`.
///
/// This maps the signed domain into unsigned space before bit-interleaving,
/// preserving Z-order across the origin.
const MORTON_BIAS: i64 = 1 << 20;

/// Valid voxel coordinate range for Morton encoding.
///
/// The bias maps `[MORTON_COORD_MIN, MORTON_COORD_MAX]` bijectively onto
/// `[0, 2^21 - 1]`, which is exactly the 21-bit domain consumed by
/// [`spread_bits`].  Coordinates outside this range alias (collide) with
/// in-range coordinates after the 21-bit mask, corrupting span membership
/// and neighbour relationships.
///
/// Floating-point inputs can produce `i32::MIN` / `i32::MAX` via the
/// saturating `as i32` cast, so all voxel coordinates must be clamped here
/// before they reach the encoding step.
pub const MORTON_COORD_MIN: i32 = -(1 << 20); // -MORTON_BIAS
pub const MORTON_COORD_MAX: i32 = (1 << 20) - 1; // MORTON_BIAS - 1

/// Spread 21 input bits so that input bit `i` occupies output bit `3*i`.
///
/// Uses the "magic bits" technique from libmorton.  All remaining output bits
/// are zero.  The three-axis Morton key is then:
///
/// ```text
/// spread(x) | (spread(y) << 1) | (spread(z) << 2)
/// ```
///
/// **SIMD extension point**: replace the scalar shifts and ORs here with
/// vector equivalents (e.g. AVX2 `_mm256_or_si256` / `_mm256_slli_epi64`)
/// applied to a register holding multiple coordinates in parallel.
#[inline]
fn spread_bits(mut x: u64) -> u64 {
    x &= 0x1fffff;
    x = (x | (x << 32)) & 0x1f00000000ffff;
    x = (x | (x << 16)) & 0x1f0000ff0000ff;
    x = (x | (x << 8)) & 0x100f00f00f00f00f;
    x = (x | (x << 4)) & 0x10c30c30c30c30c3;
    x = (x | (x << 2)) & 0x1249249249249249;
    x
}

/// Encode signed voxel coordinates into a 63-bit Morton (Z-order) key.
///
/// Coordinates must lie in `[-2^20, 2^20)`.  The bias is applied before
/// interleaving so that Z-order is preserved across negative coordinates.
#[inline]
pub fn morton_encode_voxel(vx: i32, vy: i32, vz: i32) -> u64 {
    let x = (vx.clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64;
    let y = (vy.clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64;
    let z = (vz.clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64;
    spread_bits(x) | (spread_bits(y) << 1) | (spread_bits(z) << 2)
}

/// Batch version: encode parallel voxel coordinate slices into Morton keys.
///
/// All three slices must have equal length.  The three axes are encoded in
/// **separate loops** so that a SIMD implementation can substitute the inner
/// `spread_bits` call with a vectorized kernel applied axis-by-axis.
///
/// **SIMD extension point**: replace each loop body with a vector call that
/// processes `N` coordinates per iteration using a SIMD `spread_bits_vec`.
pub fn morton_encode_batch(vx: &[i32], vy: &[i32], vz: &[i32]) -> Vec<u64> {
    let n = vx.len();
    debug_assert_eq!(n, vy.len());
    debug_assert_eq!(n, vz.len());
    let mut keys = vec![0u64; n];
    for i in 0..n {
        keys[i] = spread_bits(
            (vx[i].clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64,
        );
    }
    for i in 0..n {
        keys[i] |= spread_bits(
            (vy[i].clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64,
        ) << 1;
    }
    for i in 0..n {
        keys[i] |= spread_bits(
            (vz[i].clamp(MORTON_COORD_MIN, MORTON_COORD_MAX) as i64 + MORTON_BIAS) as u64,
        ) << 2;
    }
    keys
}

// ---------------------------------------------------------------------------
// Voxelization
// ---------------------------------------------------------------------------

/// Quantize a single floating-point coordinate to its voxel index.
///
/// Uses floor division: coordinate `c` maps to `floor(c / voxel_size)`.
/// `inv` = `1.0 / voxel_size`.
#[inline]
pub(crate) fn quantize(coord: f32, inv: f32) -> i32 {
    ((coord * inv).floor() as i32).clamp(MORTON_COORD_MIN, MORTON_COORD_MAX)
}

/// Batch-quantize SoA point arrays into voxel coordinate arrays.
///
/// `inv` = `1.0 / voxel_size`.  Each component array is processed in its own
/// loop so the compiler can auto-vectorize each axis independently.
///
/// **SIMD extension point**: replace each loop with a vectorized
/// multiply-then-floor that processes multiple floats per cycle.
pub fn voxelize_batch(
    px: &[f32],
    py: &[f32],
    pz: &[f32],
    inv: f32,
    out_vx: &mut [i32],
    out_vy: &mut [i32],
    out_vz: &mut [i32],
) {
    debug_assert_eq!(px.len(), out_vx.len());
    debug_assert_eq!(py.len(), out_vy.len());
    debug_assert_eq!(pz.len(), out_vz.len());
    for i in 0..px.len() {
        out_vx[i] = quantize(px[i], inv);
    }
    for i in 0..py.len() {
        out_vy[i] = quantize(py[i], inv);
    }
    for i in 0..pz.len() {
        out_vz[i] = quantize(pz[i], inv);
    }
}

// ---------------------------------------------------------------------------
// Runtime structures
// ---------------------------------------------------------------------------

/// A contiguous run of Morton-sorted points that all belong to the same voxel.
#[derive(Debug, Clone, Copy)]
pub struct VoxelSpan {
    /// Index of the first point in [`MortonIndex::sorted_cloud`].
    pub start: usize,
    /// Number of points in this voxel.
    pub len: usize,
    /// Morton key that identifies this voxel.
    pub morton_key: u64,
}

// ---------------------------------------------------------------------------
// Span lookup strategy (swappable for benchmarking)
// ---------------------------------------------------------------------------

/// Strategy for locating a [`VoxelSpan`] by Morton key.
///
/// Implementations may assume that `spans` is sorted by `morton_key`.
/// Swap the strategy passed to [`MortonIndex::build_with_lookup`] to
/// benchmark binary search, hash lookup, or linear scan without touching
/// the surrounding build logic.
pub trait SpanLookup {
    /// Return the index of the span whose `morton_key` equals `key`,
    /// or `None` if no such span exists.
    fn find(&self, spans: &[VoxelSpan], key: u64) -> Option<usize>;
}

/// Binary search over the sorted `voxel_spans` array.
///
/// **Complexity**: O(log V) per call, where V = number of occupied voxels.
/// This is the default strategy used by [`MortonIndex::build`].
pub struct BinarySearchLookup;

impl SpanLookup for BinarySearchLookup {
    #[inline]
    fn find(&self, spans: &[VoxelSpan], key: u64) -> Option<usize> {
        spans.binary_search_by_key(&key, |s| s.morton_key).ok()
    }
}

// ---------------------------------------------------------------------------
// MortonIndex
// ---------------------------------------------------------------------------

/// Morton-key spatial index for voxel-accelerated DBSCAN.
///
/// After construction the clustering phase uses only the four public fields.
/// All temporary build structures (`MortonRef` array, voxel coordinate
/// arrays) are dropped at the end of [`build_with_lookup`].
///
/// [`build_with_lookup`]: MortonIndex::build_with_lookup
pub struct MortonIndex {
    /// Points reordered into Morton (Z-order) sequence.
    pub sorted_cloud: PointCloud,
    /// Voxel spans, sorted ascending by `morton_key`.
    pub voxel_spans: Vec<VoxelSpan>,
    /// Maps each sorted point index to the index of its span in `voxel_spans`.
    pub point_span_lut: Vec<usize>,
    /// `neighbors[span_idx]`: indices of spans that may contain points within
    /// `eps` of any point in span `span_idx`.
    ///
    /// The inline capacity of [`NeighborList`] equals [`MAX_NEIGHBORS`], the
    /// theoretical maximum for the configured [`NEIGHBOR_RADIUS`], so heap
    /// allocation is avoided for typical sparse point clouds.
    pub neighbors: Vec<NeighborList>,
}

impl MortonIndex {
    /// Build a `MortonIndex` using binary search for neighbor span lookups.
    ///
    /// # Parameters
    /// - `cloud`: input point cloud (read-only; copied into Morton order)
    /// - `eps`: DBSCAN search radius; the voxel size is chosen as the next
    ///   representable value above `eps` so the fixed `3×3×3` neighborhood
    ///   remains sound
    pub fn build(cloud: &PointCloud, eps: f32) -> Self {
        Self::build_with_lookup(cloud, eps, &BinarySearchLookup)
    }

    /// Build a `MortonIndex` with a custom span lookup strategy.
    ///
    /// Provide a different [`SpanLookup`] implementation to benchmark
    /// alternatives (hash lookup, linear scan, …) without changing the build
    /// logic.
    pub fn build_with_lookup<L: SpanLookup>(cloud: &PointCloud, eps: f32, lookup: &L) -> Self {
        let n = cloud.len();
        let inv = (1.0_f32 / eps).next_down();

        if n == 0 {
            return Self {
                sorted_cloud: PointCloud::new(),
                voxel_spans: Vec::new(),
                point_span_lut: Vec::new(),
                neighbors: Vec::new(),
            };
        }

        // ------------------------------------------------------------------
        // Step 1 — voxelize + compute Morton references
        // ------------------------------------------------------------------
        let mut vx = vec![0i32; n];
        let mut vy = vec![0i32; n];
        let mut vz = vec![0i32; n];
        voxelize_batch(
            &cloud.vx, &cloud.vy, &cloud.vz, inv, &mut vx, &mut vy, &mut vz,
        );

        let keys = morton_encode_batch(&vx, &vy, &vz);

        // (morton_key, source_index)
        let mut morton_refs: Vec<(u64, usize)> =
            keys.into_iter().enumerate().map(|(i, k)| (k, i)).collect();

        // ------------------------------------------------------------------
        // Step 2 — sort by Morton key
        // ------------------------------------------------------------------
        morton_refs.sort_unstable_by_key(|&(key, _)| key);

        // ------------------------------------------------------------------
        // Step 3 — gather into sorted_cloud
        // ------------------------------------------------------------------
        let mut sorted_cloud = PointCloud::with_capacity(n);
        for &(_, src) in &morton_refs {
            sorted_cloud.push(cloud.vx[src], cloud.vy[src], cloud.vz[src]);
        }

        // ------------------------------------------------------------------
        // Step 4 — run-length compress into voxel_spans
        // ------------------------------------------------------------------
        let mut voxel_spans: Vec<VoxelSpan> = Vec::new();
        {
            let mut i = 0;
            while i < n {
                let key = morton_refs[i].0;
                let start = i;
                while i < n && morton_refs[i].0 == key {
                    i += 1;
                }
                voxel_spans.push(VoxelSpan {
                    start,
                    len: i - start,
                    morton_key: key,
                });
            }
        }

        // ------------------------------------------------------------------
        // Step 5 — build point → span lookup table
        // ------------------------------------------------------------------
        let mut point_span_lut = vec![0usize; n];
        for (span_idx, span) in voxel_spans.iter().enumerate() {
            for slot in point_span_lut[span.start..span.start + span.len].iter_mut() {
                *slot = span_idx;
            }
        }

        // ------------------------------------------------------------------
        // Step 6 — precompute neighbor spans
        //
        // Neighbor radius derivation:
        //   A point at position p sits in voxel floor(p/voxel_size).
        //   A neighbor at position q is in voxel floor(q/voxel_size).
        //   The voxel offset d = floor(q/voxel_size) - floor(p/voxel_size).
        //   The minimum distance between any point in voxel 0 and any point
        //   in voxel d is max(0, (d-1)*voxel_size) (1D).
        //   For this to be ≤ eps: d ≤ floor(eps/voxel_size) + 1.
        //   With voxel_size slightly larger than eps, d ≤ 1, so the fixed
        //   NEIGHBOR_RADIUS = 1 is sound.
        //
        // The representative point of a span is sorted_cloud[span.start].
        // Re-quantizing it recovers the voxel coordinates without storing
        // them in the runtime layout.
        // ------------------------------------------------------------------
        let v = voxel_spans.len();
        let mut neighbors: Vec<NeighborList> = (0..v).map(|_| NeighborList::new()).collect();

        for (span_idx, span) in voxel_spans.iter().enumerate() {
            let cx = quantize(sorted_cloud.vx[span.start], inv);
            let cy = quantize(sorted_cloud.vy[span.start], inv);
            let cz = quantize(sorted_cloud.vz[span.start], inv);

            let r = NEIGHBOR_RADIUS;
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        // saturating_add: extreme coordinates (e.g. from NaN/inf
                        // inputs to quantize) clamp rather than overflow.  The
                        // resulting key will have no matching span, so the lookup
                        // returns None and the neighbour is silently skipped.
                        let nkey = morton_encode_voxel(
                            cx.saturating_add(dx),
                            cy.saturating_add(dy),
                            cz.saturating_add(dz),
                        );
                        if let Some(idx) = lookup.find(&voxel_spans, nkey) {
                            neighbors[span_idx].push(idx);
                        }
                    }
                }
            }
        }

        // ------------------------------------------------------------------
        // Step 7 — morton_refs, vx, vy, vz are dropped here automatically.
        // ------------------------------------------------------------------
        Self {
            sorted_cloud,
            voxel_spans,
            point_span_lut,
            neighbors,
        }
    }

    /// Iterate over sorted-cloud indices of all candidate neighbors for the
    /// point at `point_idx` in the sorted cloud.
    ///
    /// These are all points in the precomputed neighboring voxel spans.
    /// **Callers must still perform exact distance checks**; this only prunes
    /// the search to a small candidate set.
    #[inline]
    pub fn candidates(&self, point_idx: usize) -> impl Iterator<Item = usize> + '_ {
        let span_idx = self.point_span_lut[point_idx];
        self.neighbors[span_idx].iter().flat_map(|&nspan| {
            let s = &self.voxel_spans[nspan];
            s.start..s.start + s.len
        })
    }
}
