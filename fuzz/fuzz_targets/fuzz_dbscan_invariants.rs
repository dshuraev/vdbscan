#![no_main]

use std::collections::BTreeSet;

use libfuzzer_sys::fuzz_target;
use vdbscan::{dbscan, Point3};

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

fn point_from_bytes(chunk: &[u8]) -> Point3 {
    let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
    let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
    let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
    Point3 { x, y, z }
}

fn parse_input(data: &[u8]) -> (Vec<Point3>, f32, usize) {
    let point_len = data.len().saturating_sub(5);
    let point_len = point_len - (point_len % 12);
    let points = data[..point_len]
        .chunks_exact(12)
        .map(point_from_bytes)
        .collect();

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

    (points, epsilon, min_pts)
}

fuzz_target!(|data: &[u8]| {
    let (points, epsilon, min_pts) = parse_input(data);
    let labels = dbscan(&points, epsilon, min_pts);

    assert_eq!(labels.len(), points.len());

    let cluster_ids: BTreeSet<usize> = labels.iter().flatten().map(|id| id.get()).collect();
    assert!(cluster_ids.iter().all(|&id| id >= 1));

    if let Some(&max_id) = cluster_ids.iter().next_back() {
        let expected: BTreeSet<usize> = (1..=max_id).collect();
        assert_eq!(cluster_ids, expected);
    }
});
