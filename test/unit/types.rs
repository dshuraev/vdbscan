use std::collections::BTreeSet;

use vdbscan::{Point3, VoxelIndex, VoxelKey};

fn point(x: f32, y: f32, z: f32) -> Point3 {
    Point3 { x, y, z }
}

fn candidate_set(index: &VoxelIndex, key: VoxelKey) -> BTreeSet<usize> {
    index.neighbors(key).collect()
}

#[test]
fn test_key_origin() {
    let points = [point(0.0, 0.0, 0.0), point(0.49, 0.49, 0.49)];
    let index = VoxelIndex::build(&points, 0.5);

    assert_eq!(index.key_of(&points[0]), VoxelKey(0, 0, 0));
    assert_eq!(index.key_of(&points[1]), VoxelKey(0, 0, 0));
}

#[test]
fn test_key_boundary() {
    let points = [point(0.5, 0.0, 0.0)];
    let index = VoxelIndex::build(&points, 0.5);

    assert_eq!(index.key_of(&points[0]), VoxelKey(1, 0, 0));
    assert_ne!(index.key_of(&points[0]), VoxelKey(0, 0, 0));
}

#[test]
fn test_key_negative_floor() {
    let points = [point(-0.1, 0.0, 0.0)];
    let index = VoxelIndex::build(&points, 0.5);

    assert_eq!(index.key_of(&points[0]), VoxelKey(-1, 0, 0));
    assert_ne!(index.key_of(&points[0]), VoxelKey(0, 0, 0));
}

#[test]
fn test_neighbors_within_epsilon() {
    let points = [point(0.0, 0.0, 0.0), point(0.4, 0.0, 0.0)];
    let index = VoxelIndex::build(&points, 0.5);

    let first_candidates = candidate_set(&index, index.key_of(&points[0]));
    let second_candidates = candidate_set(&index, index.key_of(&points[1]));

    assert!(first_candidates.contains(&1));
    assert!(second_candidates.contains(&0));
}

#[test]
fn test_all_26_neighbors_returned() {
    let mut points = Vec::with_capacity(27);
    points.push(point(0.5, 0.5, 0.5));

    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if dx == 0 && dy == 0 && dz == 0 {
                    continue;
                }
                points.push(point(dx as f32 + 0.5, dy as f32 + 0.5, dz as f32 + 0.5));
            }
        }
    }

    let index = VoxelIndex::build(&points, 1.0);
    let center_key = index.key_of(&points[0]);
    let candidates = candidate_set(&index, center_key);

    assert_eq!(center_key, VoxelKey(0, 0, 0));
    assert_eq!(candidates.len(), 27);
    assert!(candidates.contains(&0));
    assert_eq!(candidates.into_iter().skip(1).count(), 26);
}

#[test]
fn test_empty_cloud() {
    let points: [Point3; 0] = [];
    let index = VoxelIndex::build(&points, 0.5);

    assert_eq!(index.epsilon(), 0.5);
    assert_eq!(index.neighbors(VoxelKey(0, 0, 0)).count(), 0);
}
