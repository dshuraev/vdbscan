# Morton-Key Spatial Index for Voxel-Accelerated DBSCAN

## Goal

Construct a spatial index for a `PointCloud` using voxelization and Morton
(Z-order) keys to accelerate neighborhood queries for DBSCAN.

The index organizes points into contiguous voxel spans and precomputes
neighboring voxel relationships so that the clustering phase only performs
exact distance checks on a small candidate set.

The design prioritizes:

* spatial locality
* cache-friendly memory layout
* minimal runtime metadata
* one-time preprocessing cost

Original point ordering and identity preservation are **not required**.

## Input Data

The algorithm operates on the following structure:

```rust
struct PointCloud {
    vx: Vec<f32>,
    vy: Vec<f32>,
    vz: Vec<f32>,
}
```

Points are stored in **Structure-of-Arrays (SoA)** form to allow efficient
vectorized and cache-friendly access.

All arrays have length `N`.

## Build Phase

### 1. Compute Morton References

For every point:

1. Quantize the point into voxel coordinates

```
voxel_x = floor(x / voxel_size)
voxel_y = floor(y / voxel_size)
voxel_z = floor(z / voxel_size)
```

1. Encode voxel coordinates into a Morton key.

2. Emit a temporary record:

```rust
struct MortonRef {
    morton_key: u64,
    source_index: usize,
}
```

or equivalently:

```rust
Vec<(u64, usize)>
```

where:

* `morton_key` is the encoded voxel location
* `source_index` is the index of the point in the input `PointCloud`

This structure exists only to enable sorting and gathering.

#### 2. Sort by Morton Key

Sort the `MortonRef` array by `morton_key`.

After sorting:

* points belonging to the same voxel become **contiguous**
* spatial locality improves
* voxel spans can be extracted via run-length compression

#### 3. Build `sorted_cloud`

Create a new `PointCloud`:

```rust
sorted_cloud: PointCloud
```

Iterate once over the sorted `MortonRef` array and gather points:

```
sorted_cloud[i] = point_cloud[morton_refs[i].source_index]
```

After this step:

* `sorted_cloud` is Morton-ordered
* points belonging to the same voxel are contiguous

The `MortonRef` array may still be used to extract voxel spans.

#### 4. Build Voxel Spans

Scan the sorted `MortonRef` array and compress runs of identical Morton keys.

```rust
struct VoxelSpan {
    start: usize,
    len: usize,
    morton_key: u64,
}
```

where:

* `start` is the index into `sorted_cloud`
* `len` is the number of points in the voxel
* `morton_key` identifies the voxel

Collect all spans:

```rust
voxel_spans: Vec<VoxelSpan>
```

#### 5. Build Point → Span Lookup

Construct a lookup table mapping each **sorted point index** to its voxel span.

```rust
point_span_lut: Vec<usize>
```

Length = `NUMBER_OF_POINTS`.

```
point_span_lut[i] = index into voxel_spans
```

This allows constant-time mapping from a point to its voxel span.

#### 6. Precompute Neighbor Spans

For each `VoxelSpan`:

1. Take the representative point at `span.start`.

2. Recompute voxel coordinates using the same quantization rule used earlier.

3. Determine neighbor search radius:

```
neighbor_radius = ceil(eps / voxel_size)
```

Neighbor cube width:

```
cube_width = 2 * neighbor_radius + 1
```

Example:

| radius | cube size |
|  |  |
| 1      | 27        |
| 2      | 125       |

Make `neighbor_radius` a configurable (build-time) constant.

1. Enumerate all neighbor voxel coordinates in the cube.

2. Encode each neighbor voxel into a Morton key.

3. Attempt to locate the corresponding `VoxelSpan`.

Lookup strategies must be **replaceable**, such as:

* binary search on sorted `voxel_spans`
* linear scan
* hash lookup (optional)

If a span exists, record its index.

Store results:

```rust
neighbors: Vec<SmallVec<[usize; MAX_NEIGHBORS]>>
```

Where:

```
neighbors[span_index] -> list of neighboring voxel span indices
```

#### 7. Drop Temporary Structures

After building:

* `sorted_cloud`
* `voxel_spans`
* `point_span_lut`
* `neighbors`

the temporary `MortonRef` array can be discarded.

## Runtime Structures

The clustering phase uses only:

```rust
sorted_cloud: PointCloud
voxel_spans: Vec<VoxelSpan>
point_span_lut: Vec<usize>
neighbors: Vec<SmallVec<[usize; MAX_NEIGHBORS]>>
```

No voxel coordinates or Morton references need to persist.

## Clustering Phase

Iterate over sorted points.

For point `i`:

```
span_idx = point_span_lut[i]
```

Retrieve candidate voxel spans:

```
candidate_spans = neighbors[span_idx]
```

For each candidate span:

```
span = voxel_spans[candidate_span]
points = sorted_cloud[span.start .. span.start + span.len]
```

Compute exact Euclidean distances against those points.

Only these points require precise distance evaluation.

## Performance Properties

### Spatial Locality

Morton sorting clusters spatially nearby voxels and points,
improving cache efficiency.

### Reduced Memory Access

Exact distance calculations are restricted to candidate spans rather
than the full dataset.

### Minimal Runtime Metadata

Only span boundaries and neighbor lists are retained.

### Build-Time Cost

Voxelization, Morton encoding, sorting, and neighbor discovery
occur once during index construction.

## Design Constraints

* Morton encoding must be deterministic and consistent across build steps.
* Quantization must use the same formula everywhere.
* Lookup strategy for voxel spans must be swappable and benchmarkable.
* Original point ordering is intentionally discarded.
* Exact distance checks remain the final correctness guarantee.

## Complexity Analysis

### Build Phase

Let:

* `N` = number of points
* `V` = number of occupied voxels
* `K` = number of neighbor voxels inspected per occupied voxel

Then the preprocessing cost is approximately:

* voxelization + Morton encoding: `O(N)`
* sorting Morton references: `O(N log N)`
* building `sorted_cloud`: `O(N)`
* run-length compression into `voxel_spans`: `O(N)`
* building `point_span_lut`: `O(N)`
* neighbor span precomputation: `O(V * K * lookup_cost)`

Where `lookup_cost` depends on the chosen span lookup strategy:

* binary search: `O(log V)`
* linear scan: `O(V)` worst case
* hash lookup: approximately `O(1)` average

For binary search, neighbor precompute is:

```text
O(V * K * log V)
```

Typical values of `K`:

* `27` for radius 1
* `125` for radius 2

### Clustering Phase

The clustering phase does not perform global spatial search.

For each point, it only inspects points belonging to candidate voxel spans:

```text
O(total exact distance checks over candidate spans)
```

The exact cost depends on:

* voxel size
* `eps`
* point density
* span occupancy distribution

In practice, the Morton-indexed design reduces the candidate set enough that
memory access and exact distance evaluation dominate runtime.

## Memory Layout

### Input Layout

```rust
struct PointCloud {
    vx: Vec<f32>,
    vy: Vec<f32>,
    vz: Vec<f32>,
}
```

This SoA layout is preferred because it:

* improves sequential memory access
* works well with SIMD loads
* is easy to parallelize in chunks

### Temporary Build Structures

```rust
struct MortonRef {
    morton_key: u64,
    source_index: usize,
}
```

This array is used only during:

* sorting
* gathering into `sorted_cloud`
* extracting voxel spans

It can be discarded after build.

### Runtime Structures

```rust
struct VoxelSpan {
    start: usize,
    len: usize,
    morton_key: u64,
}
```

```rust
sorted_cloud: PointCloud
voxel_spans: Vec<VoxelSpan>
point_span_lut: Vec<usize>
neighbors: Vec<SmallVec<[usize; MAX_NEIGHBORS]>>
```

The hot path operates primarily on contiguous slices of `sorted_cloud`.

## Parallelization Strategy

### SIMD / CPU Parallelism

The following stages are naturally batch-oriented and suitable for SIMD or
multi-threaded CPU execution:

### Voxelization

Transform point coordinates into voxel coordinates in batch form.

### Morton Encoding

Encode quantized voxel coordinates into Morton keys in batch form.

### Gather into `sorted_cloud`

After sorting Morton references, reorder point arrays into Morton order.

These stages should be implemented as slice-based transforms rather than
single-point APIs.

Recommended shape:

```rust
fn process_batch(input: &[...], output: &mut [...])
```

This allows:

* scalar reference implementations
* chunked CPU execution
* explicit SIMD kernels
* GPU kernel mapping later

## GPU-Oriented Design Notes

The design already matches a GPU-friendly dataflow:

1. read SoA point arrays
2. quantize points
3. compute Morton keys
4. sort key/index pairs
5. gather sorted points
6. compress runs into spans

The most GPU-friendly stages are:

* quantization
* Morton encoding
* permutation / gather
* run-length compression

Neighbor precompute may also be ported later, but it is less regular.

## API Design Guidance

Helper functions should be batch-first, with scalar wrappers only for tests or
reference implementations.

Preferred pattern:

```rust
fn voxelize_batch(...)
fn morton_encode_batch(...)
```

Not:

```rust
fn voxelize_one(...)
fn morton_encode_one(...)
```

as the primary abstraction.

## Implementation Notes

### Span Lookup Strategy Must Be Swappable

Neighbor span lookup should be abstracted behind a replaceable strategy.

Examples:

* binary search over sorted `voxel_spans`
* linear scan
* temporary hash-based lookup

This is important because the best strategy depends on:

* `V`
* neighbor radius
* cache behavior
* benchmark results

### Representative Point Re-Quantization

Voxel coordinates do not need to be stored permanently.

During neighbor precompute, use the first point of each span (`start`) as a
representative point and re-quantize it to recover voxel coordinates.

This avoids carrying extra voxel-coordinate metadata into the runtime layout.

## Runtime Correctness

Voxel spans and neighbor lists only generate candidate points.

Final DBSCAN connectivity decisions must still be based on exact distance
checks.

## Summary

Pipeline:

```text
PointCloud
   ↓ voxelize + morton encode
MortonRef[(key, source_index)]
   ↓ sort
MortonRef sorted
   ↓ gather
sorted_cloud
   ↓ run-length compress
voxel_spans
   ↓
point_span_lut
   ↓ neighbor enumeration
neighbors
```

Clustering then operates only on:

```text
sorted_cloud
voxel_spans
neighbors
point_span_lut
```

This structure minimizes random memory access while maintaining correct DBSCAN semantics.
