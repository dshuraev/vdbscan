use std::num::NonZeroUsize;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3 {
    #[inline]
    pub fn distance_sq(self, other: Point3) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }
}

/// A point cloud with components stored in separate contiguous vectors (SoA layout).
#[derive(Debug, Clone, PartialEq)]
pub struct PointCloud {
    pub vx: Vec<f32>,
    pub vy: Vec<f32>,
    pub vz: Vec<f32>,
}

impl PointCloud {
    pub fn new() -> Self {
        Self {
            vx: Vec::new(),
            vy: Vec::new(),
            vz: Vec::new(),
        }
    }

    pub fn with_capacity(n: usize) -> Self {
        Self {
            vx: Vec::with_capacity(n),
            vy: Vec::with_capacity(n),
            vz: Vec::with_capacity(n),
        }
    }

    pub fn push(&mut self, x: f32, y: f32, z: f32) {
        self.vx.push(x);
        self.vy.push(y);
        self.vz.push(z);
    }

    pub fn len(&self) -> usize {
        self.vx.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vx.is_empty()
    }

    pub fn get(&self, i: usize) -> Point3 {
        Point3 {
            x: self.vx[i],
            y: self.vy[i],
            z: self.vz[i],
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = Point3> + '_ {
        (0..self.len()).map(move |i| self.get(i))
    }
}

impl Default for PointCloud {
    fn default() -> Self {
        Self::new()
    }
}

/// `None` = noise, `Some(id)` = cluster id (1-based).
pub type ClusterLabel = Option<NonZeroUsize>;

/// Result of a DBSCAN run: a point cloud and a parallel vector of cluster labels.
/// `cloud[i]` and `labels[i]` correspond to the same point.
/// The ordering may differ from the input `PointCloud`.
#[derive(Debug, Clone, PartialEq)]
pub struct Clustering {
    pub cloud: PointCloud,
    pub labels: Vec<ClusterLabel>,
}

impl Clustering {
    pub fn len(&self) -> usize {
        self.labels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.labels.is_empty()
    }

    /// Iterate over `(point, label)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (Point3, ClusterLabel)> + '_ {
        self.cloud.iter().zip(self.labels.iter().copied())
    }

    /// Number of distinct cluster IDs (noise excluded).
    pub fn cluster_count(&self) -> usize {
        use std::collections::HashSet;
        self.labels.iter().flatten().collect::<HashSet<_>>().len()
    }

    /// Number of noise points (label is `None`).
    pub fn noise_count(&self) -> usize {
        self.labels.iter().filter(|l| l.is_none()).count()
    }
}
