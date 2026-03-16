use std::collections::VecDeque;
use std::num::NonZeroUsize;

use crate::morton::MortonIndex;
use crate::types::{ClusterLabel, Clustering, PointCloud};

pub fn dbscan(cloud: &PointCloud, epsilon: f32, min_pts: usize) -> Clustering {
    assert!(epsilon > 0.0, "epsilon must be positive, got {epsilon}");

    let n = cloud.len();
    let index = MortonIndex::build(cloud, epsilon);
    let epsilon_sq = epsilon * epsilon;
    let mut labels: Vec<ClusterLabel> = vec![None; n];
    let mut visited = vec![false; n];

    // First pass: mark core points.
    let mut is_core = vec![false; n];
    for (i, core) in is_core.iter_mut().enumerate() {
        let p = index.sorted_cloud.get(i);
        let count = index
            .candidates(i)
            .filter(|&nb| nb != i)
            .filter(|&nb| index.sorted_cloud.get(nb).distance_sq(p) <= epsilon_sq)
            .count();
        if count >= min_pts {
            *core = true;
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
            let p = index.sorted_cloud.get(cur);
            for nb in index.candidates(cur) {
                if !visited[nb] && index.sorted_cloud.get(nb).distance_sq(p) <= epsilon_sq {
                    visited[nb] = true;
                    labels[nb] = Some(cluster);
                    if is_core[nb] {
                        queue.push_back(nb);
                    }
                }
            }
        }
    }

    Clustering {
        cloud: index.sorted_cloud,
        labels,
    }
}
