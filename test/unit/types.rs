use std::num::NonZeroUsize;
use vdbscan::{ClusterLabel, Clustering, Point3, PointCloud};

// ---------------------------------------------------------------------------
// Point3
// ---------------------------------------------------------------------------

#[test]
fn test_distance_sq_zero() {
    let p = Point3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    assert_eq!(p.distance_sq(p), 0.0);
}

#[test]
fn test_distance_sq_axis_aligned() {
    let a = Point3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let b = Point3 {
        x: 3.0,
        y: 0.0,
        z: 0.0,
    };
    assert_eq!(a.distance_sq(b), 9.0);
}

#[test]
fn test_distance_sq_symmetric() {
    let a = Point3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let b = Point3 {
        x: 4.0,
        y: 6.0,
        z: 3.0,
    };
    assert_eq!(a.distance_sq(b), b.distance_sq(a));
}

// ---------------------------------------------------------------------------
// PointCloud
// ---------------------------------------------------------------------------

#[test]
fn test_cloud_push_and_get() {
    let mut cloud = PointCloud::new();
    cloud.push(1.0, 2.0, 3.0);
    cloud.push(-1.0, 0.5, 7.0);

    assert_eq!(cloud.len(), 2);
    assert_eq!(
        cloud.get(0),
        Point3 {
            x: 1.0,
            y: 2.0,
            z: 3.0
        }
    );
    assert_eq!(
        cloud.get(1),
        Point3 {
            x: -1.0,
            y: 0.5,
            z: 7.0
        }
    );
}

#[test]
fn test_cloud_is_empty() {
    let cloud = PointCloud::new();
    assert!(cloud.is_empty());

    let mut cloud = PointCloud::with_capacity(4);
    assert!(cloud.is_empty());
    cloud.push(0.0, 0.0, 0.0);
    assert!(!cloud.is_empty());
}

#[test]
fn test_cloud_iter_matches_get() {
    let mut cloud = PointCloud::new();
    for i in 0..5 {
        cloud.push(i as f32, i as f32 * 2.0, 0.0);
    }
    let from_iter: Vec<Point3> = cloud.iter().collect();
    let from_get: Vec<Point3> = (0..cloud.len()).map(|i| cloud.get(i)).collect();
    assert_eq!(from_iter, from_get);
}

// ---------------------------------------------------------------------------
// Clustering
// ---------------------------------------------------------------------------

fn make_clustering(pts: &[(f32, f32, f32)], labels: &[Option<usize>]) -> Clustering {
    let mut cloud = PointCloud::with_capacity(pts.len());
    for &(x, y, z) in pts {
        cloud.push(x, y, z);
    }
    let labels = labels
        .iter()
        .map(|&id| id.and_then(NonZeroUsize::new))
        .collect();
    Clustering { cloud, labels }
}

#[test]
fn test_clustering_len() {
    let c = make_clustering(&[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], &[Some(1), None]);
    assert_eq!(c.len(), 2);
}

#[test]
fn test_clustering_cluster_count() {
    let c = make_clustering(
        &[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
        &[Some(1), Some(1), Some(2)],
    );
    assert_eq!(c.cluster_count(), 2);
}

#[test]
fn test_clustering_noise_count() {
    let c = make_clustering(
        &[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
        &[Some(1), None, None],
    );
    assert_eq!(c.noise_count(), 2);
}

#[test]
fn test_clustering_iter_correspondence() {
    let pts = [(0.0f32, 0.0, 0.0), (1.0, 2.0, 3.0)];
    let c = make_clustering(&pts, &[Some(1), Some(2)]);
    let pairs: Vec<(Point3, ClusterLabel)> = c.iter().collect();
    assert_eq!(
        pairs[0].0,
        Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    );
    assert_eq!(
        pairs[1].0,
        Point3 {
            x: 1.0,
            y: 2.0,
            z: 3.0
        }
    );
}
