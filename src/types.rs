use std::num::NonZeroUsize;

use ahash::AHashMap;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3 {
    #[inline]
    pub fn distance(self, other: Point3) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoxelKey(pub i32, pub i32, pub i32);

/// `None` = noise, `Some(id)` = cluster id (1-based)
pub type ClusterLabel = Option<NonZeroUsize>;

pub struct VoxelIndex {
    voxel_size: f32,
    /// indices into the original `points` slice, grouped by voxel cell
    cells: AHashMap<VoxelKey, Vec<usize>>,
}

// Offsets for all 27 cells in a 3×3×3 cube around a key.
const NEIGHBOR_OFFSETS: [(i32, i32, i32); 27] = {
    let mut out = [(0i32, 0i32, 0i32); 27];
    let mut i = 0;
    let mut dx = -1i32;
    while dx <= 1 {
        let mut dy = -1i32;
        while dy <= 1 {
            let mut dz = -1i32;
            while dz <= 1 {
                out[i] = (dx, dy, dz);
                i += 1;
                dz += 1;
            }
            dy += 1;
        }
        dx += 1;
    }
    out
};

impl VoxelIndex {
    pub fn build(points: &[Point3], voxel_size: f32) -> Self {
        let inv = 1.0 / voxel_size;
        let mut cells: AHashMap<VoxelKey, Vec<usize>> =
            AHashMap::with_capacity(points.len() / 4 + 1);

        for (i, p) in points.iter().enumerate() {
            let key = VoxelKey(
                (p.x * inv).floor() as i32,
                (p.y * inv).floor() as i32,
                (p.z * inv).floor() as i32,
            );
            cells.entry(key).or_default().push(i);
        }

        Self { voxel_size, cells }
    }

    #[inline]
    pub fn epsilon(&self) -> f32 {
        self.voxel_size
    }

    pub fn key_of(&self, p: &Point3) -> VoxelKey {
        let inv = 1.0 / self.voxel_size;
        VoxelKey(
            (p.x * inv).floor() as i32,
            (p.y * inv).floor() as i32,
            (p.z * inv).floor() as i32,
        )
    }

    /// Iterate over the indices of every point in the 3×3×3 neighborhood
    /// of `key` (27 cells, including `key` itself).
    pub fn neighbors(&self, key: VoxelKey) -> impl Iterator<Item = usize> + '_ {
        let VoxelKey(kx, ky, kz) = key;
        NEIGHBOR_OFFSETS.iter().flat_map(move |&(dx, dy, dz)| {
            let neighbor = VoxelKey(kx + dx, ky + dy, kz + dz);
            self.cells
                .get(&neighbor)
                .map(Vec::as_slice)
                .unwrap_or(&[])
                .iter()
                .copied()
        })
    }
}
