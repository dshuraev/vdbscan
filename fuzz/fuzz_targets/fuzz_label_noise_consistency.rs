#![no_main]

use libfuzzer_sys::fuzz_target;
use vdbscan::{Point3, PointCloud, dbscan};

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
    if cloud.len() > 500 {
        return;
    }
    if cloud.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) {
        return;
    }

    let clustering = dbscan(&cloud, epsilon, min_pts);
    let dist_sq = epsilon * epsilon;

    // Collect output points for neighbor counting.
    let points: Vec<Point3> = clustering.cloud.iter().collect();

    for (i, (p, label)) in clustering.iter().enumerate() {
        if label.is_some() {
            continue;
        }

        let neighbor_count = points
            .iter()
            .enumerate()
            .filter(|&(j, &other)| j != i && p.distance_sq(other) <= dist_sq)
            .count();

        if neighbor_count >= min_pts {
            eprintln!("epsilon={epsilon}, min_pts={min_pts}");
            eprintln!("points ({}):", points.len());
            for (idx, pt) in points.iter().enumerate() {
                eprintln!("  [{idx}] ({}, {}, {})", pt.x, pt.y, pt.z);
            }
            eprintln!("noise point {i} has {neighbor_count} true neighbors");
            for (j, &other) in points.iter().enumerate() {
                if j != i {
                    let d = p.distance_sq(other);
                    if d <= dist_sq {
                        eprintln!("  neighbor [{j}] dist={d}");
                    }
                }
            }
        }

        assert!(
            neighbor_count < min_pts,
            "noise point at index {i} has {neighbor_count} neighbors, min_pts={min_pts}"
        );
    }
});
