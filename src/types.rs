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
    pub fn distance_sq(self, other: Point3) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoxelKey(pub i32, pub i32, pub i32);

/// `None` = noise, `Some(id)` = cluster id (1-based)
pub type ClusterLabel = Option<NonZeroUsize>;

pub struct VoxelIndex {
    voxel_size: f32,
    inv: f32,
    /// indices into the original `points` slice, grouped by voxel cell
    cells: AHashMap<VoxelKey, Vec<usize>>,
}

// Offsets for all 125 cells in a 5×5×5 cube around a key (±2 in each axis).
const NEIGHBOR_OFFSETS: [(i32, i32, i32); 125] = {
    let mut out = [(0i32, 0i32, 0i32); 125];
    let mut i = 0;
    let mut dx = -2i32;
    while dx <= 2 {
        let mut dy = -2i32;
        while dy <= 2 {
            let mut dz = -2i32;
            while dz <= 2 {
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

        Self {
            voxel_size,
            inv,
            cells,
        }
    }

    #[inline]
    pub fn epsilon(&self) -> f32 {
        self.voxel_size
    }

    pub fn key_of(&self, p: &Point3) -> VoxelKey {
        VoxelKey(
            (p.x * self.inv).floor() as i32,
            (p.y * self.inv).floor() as i32,
            (p.z * self.inv).floor() as i32,
        )
    }

    /// Iterate over the indices of every point in the 5×5×5 neighborhood
    /// of `key` (125 cells, including `key` itself).
    pub fn neighbors(&self, key: VoxelKey) -> impl Iterator<Item = usize> + '_ {
        let VoxelKey(kx, ky, kz) = key;
        NEIGHBOR_OFFSETS.iter().flat_map(move |&(dx, dy, dz)| {
            let neighbor = VoxelKey(
                kx.wrapping_add(dx),
                ky.wrapping_add(dy),
                kz.wrapping_add(dz),
            );
            self.cells
                .get(&neighbor)
                .map(Vec::as_slice)
                .unwrap_or(&[])
                .iter()
                .copied()
        })
    }
}
