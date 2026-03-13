use std::collections::VecDeque;
use std::num::NonZeroUsize;

use crate::types::{ClusterLabel, Point3, VoxelIndex};

pub fn dbscan(points: &[Point3], epsilon: f32, min_pts: usize) -> Vec<ClusterLabel> {
    assert!(epsilon > 0.0, "epsilon must be positive, got {epsilon}");

    let index = VoxelIndex::build(points, epsilon);
    let n = points.len();
    let mut labels: Vec<ClusterLabel> = vec![None; n];
    let mut visited = vec![false; n];

    // First pass: mark core points.
    let mut is_core = vec![false; n];
    for (i, p) in points.iter().enumerate() {
        let count = index
            .neighbors(index.key_of(p))
            .filter(|&nb| nb != i && points[nb].distance(*p) <= epsilon)
            .count();
        if count >= min_pts {
            is_core[i] = true;
        }
    }

    // Second pass: BFS from each unvisited core point.
    let core_count = is_core.iter().filter(|&&c| c).count();
    let mut queue: VecDeque<usize> = VecDeque::with_capacity(core_count);
    let mut next_id: usize = 1;

    for i in 0..n {
        if !is_core[i] || visited[i] {
            continue;
        }

        let cluster = NonZeroUsize::new(next_id).unwrap();
        next_id += 1;

        visited[i] = true;
        labels[i] = Some(cluster);
        queue.push_back(i);

        while let Some(cur) = queue.pop_front() {
            let key = index.key_of(&points[cur]);
            for nb in index.neighbors(key) {
                if !visited[nb] && points[nb].distance(points[cur]) <= epsilon {
                    visited[nb] = true;
                    labels[nb] = Some(cluster);
                    if is_core[nb] {
                        queue.push_back(nb);
                    }
                }
            }
        }
    }

    labels
}
