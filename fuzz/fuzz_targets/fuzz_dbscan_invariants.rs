#![no_main]

use std::collections::BTreeSet;

use libfuzzer_sys::fuzz_target;
use vdbscan::{PointCloud, dbscan};

fn clamp_epsilon(raw: f32) -> f32 {
    if raw.is_finite() {
        raw.clamp(0.05, 5.0)
    } else {
        0.5
    }
}

fn clamp_min_pts(raw: u8) -> usize {
    usize::from(raw).clamp(1, 20)
}

fn parse_input(data: &[u8]) -> (PointCloud, f32, usize) {
    let point_len = data.len().saturating_sub(5);
    let point_len = point_len - (point_len % 12);
    let mut cloud = PointCloud::with_capacity(point_len / 12);
    for chunk in data[..point_len].chunks_exact(12) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
        cloud.push(x, y, z);
    }

    let epsilon = if data.len() >= point_len + 4 {
        clamp_epsilon(f32::from_le_bytes(
            data[point_len..point_len + 4].try_into().unwrap(),
        ))
    } else {
        0.5
    };

    let min_pts = data
        .get(point_len + 4)
        .copied()
        .map(clamp_min_pts)
        .unwrap_or(5);

    (cloud, epsilon, min_pts)
}

fuzz_target!(|data: &[u8]| {
    let (cloud, epsilon, min_pts) = parse_input(data);
    let clustering = dbscan(cloud, epsilon, min_pts);

    let cluster_ids: BTreeSet<usize> =
        clustering.labels.iter().flatten().map(|id| id.get()).collect();
    assert!(cluster_ids.iter().all(|&id| id >= 1));

    if let Some(&max_id) = cluster_ids.iter().next_back() {
        let expected: BTreeSet<usize> = (1..=max_id).collect();
        assert_eq!(cluster_ids, expected);
    }
});
