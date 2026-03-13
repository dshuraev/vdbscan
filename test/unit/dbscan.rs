use std::collections::BTreeSet;

use vdbscan::{Point3, dbscan};

fn point(x: f32, y: f32, z: f32) -> Point3 {
    Point3 { x, y, z }
}

fn line_points(count: usize, spacing: f32, origin_x: f32) -> Vec<Point3> {
    (0..count)
        .map(|i| point(origin_x + i as f32 * spacing, 0.0, 0.0))
        .collect()
}

fn dense_blob(origin_x: f32) -> Vec<Point3> {
    let mut points = Vec::with_capacity(20);
    for row in 0..4 {
        for col in 0..5 {
            points.push(point(origin_x + col as f32 * 0.1, row as f32 * 0.1, 0.0));
        }
    }
    points
}

#[test]
fn test_three_blobs() {
    let mut points = Vec::with_capacity(60);
    points.extend(dense_blob(0.0));
    points.extend(dense_blob(5.0));
    points.extend(dense_blob(10.0));

    let labels = dbscan(&points, 0.5, 5);
    let cluster_ids: BTreeSet<usize> = labels.iter().flatten().map(|id| id.get()).collect();

    assert_eq!(labels.len(), points.len());
    assert_eq!(cluster_ids.len(), 3);
    assert_eq!(labels.iter().filter(|label| label.is_none()).count(), 0);
}

#[test]
fn test_single_isolated_point() {
    let points = [point(0.0, 0.0, 0.0)];
    let labels = dbscan(&points, 0.5, 2);

    assert_eq!(labels, vec![None]);
}

#[test]
fn test_border_point() {
    let mut points = vec![
        point(0.0, 0.0, 0.0),
        point(0.0, 0.2, 0.0),
        point(0.2, 0.0, 0.0),
        point(0.2, 0.2, 0.0),
        point(0.1, 0.1, 0.0),
    ];
    points.push(point(0.6, 0.1, 0.0));

    let labels = dbscan(&points, 0.5, 5);
    let border_label = labels[5];

    assert!(border_label.is_some());
    assert_eq!(border_label, labels[4]);
}

#[test]
fn test_noise_point() {
    let mut points = dense_blob(0.0);
    points.push(point(2.0, 2.0, 2.0));

    let labels = dbscan(&points, 0.5, 5);

    assert!(labels[..20].iter().all(|label| label.is_some()));
    assert_eq!(labels[20], None);
}

#[test]
fn test_no_duplicate_cluster_ids_from_adjacent_cores() {
    let points = line_points(6, 0.2, 0.0);
    let labels = dbscan(&points, 0.5, 2);

    assert!(labels.iter().all(|label| label.is_some()));
    assert_eq!(labels[0], labels[1]);
}

#[test]
fn test_min_pts_one() {
    let points = line_points(10, 2.0, 0.0);
    let labels = dbscan(&points, 0.5, 1);

    assert_eq!(labels.len(), 10);
    assert!(labels.iter().all(|label| label.is_some()));
}

#[test]
#[cfg_attr(miri, ignore)]
fn test_large_chain_no_stack_overflow() {
    let points = line_points(10_000, 0.3, 0.0);
    let labels = dbscan(&points, 0.5, 2);

    assert_eq!(labels.len(), points.len());
    assert!(labels.iter().all(|label| label.is_some()));
    assert_eq!(labels[0], labels[points.len() - 1]);
}

#[test]
fn test_output_length() {
    let points = dense_blob(0.0);
    let labels = dbscan(&points, 0.5, 5);

    assert_eq!(labels.len(), points.len());
}

#[test]
fn test_cluster_ids_are_one_based() {
    let points = dense_blob(0.0);
    let labels = dbscan(&points, 0.5, 5);

    let min_cluster_id = labels.iter().flatten().map(|id| id.get()).min();
    assert_eq!(min_cluster_id, Some(1));
}

#[test]
fn test_empty_input() {
    let points: [Point3; 0] = [];
    let labels = dbscan(&points, 0.5, 5);

    assert!(labels.is_empty());
}

#[test]
fn test_determinism() {
    let mut points = dense_blob(0.0);
    points.extend(dense_blob(5.0));

    let first = dbscan(&points, 0.5, 5);
    let second = dbscan(&points, 0.5, 5);

    assert_eq!(first, second);
}
