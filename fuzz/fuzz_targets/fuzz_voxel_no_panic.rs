#![no_main]

use libfuzzer_sys::fuzz_target;
use vdbscan::morton::MortonIndex;
use vdbscan::PointCloud;

fn clamp_eps(raw: f32) -> f32 {
    if raw.is_finite() {
        raw.clamp(0.05, 5.0)
    } else {
        0.5
    }
}

fuzz_target!(|data: &[u8]| {
    // First 4 bytes: epsilon; remaining bytes: points (12 bytes each).
    let (eps_bytes, point_bytes) = data.split_at(data.len().min(4));
    let eps = if eps_bytes.len() == 4 {
        clamp_eps(f32::from_le_bytes(eps_bytes.try_into().unwrap()))
    } else {
        0.5
    };

    let mut cloud = PointCloud::with_capacity(point_bytes.len() / 12);
    for chunk in point_bytes.chunks_exact(12) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
        cloud.push(x, y, z);
    }

    let n = cloud.len();
    let index = MortonIndex::build(cloud, eps);

    // Exercise the candidates iterator for every point; must not panic.
    for i in 0..n {
        let _ = index.candidates(i).count();
    }
});
