use std::collections::{BTreeSet, HashMap};

use vdbscan::{ClusterLabel, Clustering, PointCloud, dbscan};

fn cloud_from_xyz(pts: &[(f32, f32, f32)]) -> PointCloud {
    let mut cloud = PointCloud::with_capacity(pts.len());
    for &(x, y, z) in pts {
        cloud.push(x, y, z);
    }
    cloud
}

fn line_cloud(count: usize, spacing: f32, origin_x: f32) -> PointCloud {
    let mut cloud = PointCloud::with_capacity(count);
    for i in 0..count {
        cloud.push(origin_x + i as f32 * spacing, 0.0, 0.0);
    }
    cloud
}

fn dense_blob_cloud(origin_x: f32) -> PointCloud {
    let mut cloud = PointCloud::with_capacity(20);
    for row in 0..4 {
        for col in 0..5 {
            cloud.push(origin_x + col as f32 * 0.1, row as f32 * 0.1, 0.0);
        }
    }
    cloud
}

/// Returns the label for the unique point matching `(x, y, z)`.
/// Panics if not found.
fn label_at(clustering: &Clustering, x: f32, y: f32, z: f32) -> ClusterLabel {
    clustering
        .iter()
        .find(|(p, _)| p.x == x && p.y == y && p.z == z)
        .map(|(_, l)| l)
        .unwrap_or_else(|| panic!("point ({x}, {y}, {z}) not found in clustering"))
}

/// Canonical cluster representation: a set of sets of point coordinates (as bits),
/// independent of label IDs and point ordering.
fn cluster_sets(clustering: &Clustering) -> BTreeSet<BTreeSet<(u32, u32, u32)>> {
    let mut map: HashMap<usize, BTreeSet<(u32, u32, u32)>> = HashMap::new();
    for (p, label) in clustering.iter() {
        if let Some(id) = label {
            map.entry(id.get())
                .or_default()
                .insert((p.x.to_bits(), p.y.to_bits(), p.z.to_bits()));
        }
    }
    map.into_values().collect()
}

#[test]
fn test_three_blobs() {
    let mut cloud = dense_blob_cloud(0.0);
    for p in dense_blob_cloud(5.0).iter() {
        cloud.push(p.x, p.y, p.z);
    }
    for p in dense_blob_cloud(10.0).iter() {
        cloud.push(p.x, p.y, p.z);
    }

    let clustering = dbscan(&cloud, 0.5, 5);

    assert_eq!(clustering.len(), 60);
    assert_eq!(clustering.cluster_count(), 3);
    assert_eq!(clustering.noise_count(), 0);
}

#[test]
fn test_single_isolated_point() {
    let cloud = cloud_from_xyz(&[(0.0, 0.0, 0.0)]);
    let clustering = dbscan(&cloud, 0.5, 2);

    assert_eq!(clustering.len(), 1);
    assert_eq!(clustering.noise_count(), 1);
    assert_eq!(clustering.cluster_count(), 0);
}

#[test]
fn test_border_point() {
    let cloud = cloud_from_xyz(&[
        (0.0, 0.0, 0.0),
        (0.0, 0.2, 0.0),
        (0.2, 0.0, 0.0),
        (0.2, 0.2, 0.0),
        (0.1, 0.1, 0.0),
        (0.6, 0.1, 0.0),
    ]);

    let clustering = dbscan(&cloud, 0.5, 5);

    let core_label = label_at(&clustering, 0.1, 0.1, 0.0);
    let border_label = label_at(&clustering, 0.6, 0.1, 0.0);

    assert!(
        border_label.is_some(),
        "border point should be in a cluster"
    );
    assert_eq!(
        border_label, core_label,
        "border and core should share a cluster"
    );
}

#[test]
fn test_noise_point() {
    let mut cloud = dense_blob_cloud(0.0);
    cloud.push(2.0, 2.0, 2.0);

    let clustering = dbscan(&cloud, 0.5, 5);

    let noise_label = label_at(&clustering, 2.0, 2.0, 2.0);
    assert_eq!(noise_label, None, "isolated point should be noise");
    assert_eq!(clustering.noise_count(), 1);
    assert_eq!(clustering.cluster_count(), 1);
}

#[test]
fn test_no_duplicate_cluster_ids_from_adjacent_cores() {
    let cloud = line_cloud(6, 0.2, 0.0);
    let clustering = dbscan(&cloud, 0.5, 2);

    assert_eq!(
        clustering.cluster_count(),
        1,
        "all points should form one cluster"
    );
    assert_eq!(clustering.noise_count(), 0);
}

#[test]
#[cfg_attr(miri, ignore)]
fn test_large_chain_no_stack_overflow() {
    let cloud = line_cloud(10_000, 0.3, 0.0);
    let clustering = dbscan(&cloud, 0.5, 2);

    assert_eq!(clustering.len(), 10_000);
    assert_eq!(clustering.cluster_count(), 1);
    assert_eq!(clustering.noise_count(), 0);
}

#[test]
fn test_output_length() {
    let cloud = dense_blob_cloud(0.0);
    let n = cloud.len();
    let clustering = dbscan(&cloud, 0.5, 5);

    assert_eq!(clustering.len(), n);
}

#[test]
fn test_cluster_ids_are_one_based() {
    let cloud = dense_blob_cloud(0.0);
    let clustering = dbscan(&cloud, 0.5, 5);

    let min_id = clustering.labels.iter().flatten().map(|id| id.get()).min();
    assert_eq!(min_id, Some(1));
}

#[test]
fn test_empty_input() {
    let cloud = PointCloud::new();
    let clustering = dbscan(&cloud, 0.5, 5);

    assert!(clustering.is_empty());
}

#[test]
fn test_determinism() {
    let build = || {
        let mut cloud = dense_blob_cloud(0.0);
        for p in dense_blob_cloud(5.0).iter() {
            cloud.push(p.x, p.y, p.z);
        }
        cloud
    };

    let first_cloud = build();
    let second_cloud = build();
    let first = dbscan(&first_cloud, 0.5, 5);
    let second = dbscan(&second_cloud, 0.5, 5);

    assert_eq!(cluster_sets(&first), cluster_sets(&second));
}
