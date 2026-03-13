#![no_main]

use libfuzzer_sys::fuzz_target;
use vdbscan::{Point3, VoxelIndex};

fn clamp_voxel_size(raw: f32) -> f32 {
    if raw.is_finite() {
        raw.clamp(0.05, 5.0)
    } else {
        0.5
    }
}

fn point_from_bytes(chunk: &[u8]) -> Point3 {
    let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
    let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
    let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
    Point3 { x, y, z }
}

fn parse_points(bytes: &[u8]) -> Vec<Point3> {
    bytes.chunks_exact(12).map(point_from_bytes).collect()
}

fuzz_target!(|data: &[u8]| {
    let (voxel_bytes, point_bytes) = data.split_at(data.len().min(4));
    let voxel_size = if voxel_bytes.len() == 4 {
        clamp_voxel_size(f32::from_le_bytes(voxel_bytes.try_into().unwrap()))
    } else {
        0.5
    };

    let points = parse_points(point_bytes);
    let index = VoxelIndex::build(&points, voxel_size);

    for point in &points {
        let _ = index.key_of(point);
    }
});
