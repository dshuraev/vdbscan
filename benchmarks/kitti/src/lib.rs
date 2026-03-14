use std::collections::VecDeque;
use std::fmt;
use std::fs;
use std::io;
use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};

use kiddo::{SquaredEuclidean, float::kdtree::KdTree as KiddoKdTree};
use vdbscan::{ClusterLabel, Point3};

pub const DEFAULT_EPSILON: f32 = 0.4;
pub const DEFAULT_MIN_PTS: usize = 5;
pub const KITTI_PATH_ENV: &str = "VDBSCAN_KITTI_PATH";
pub const KITTI_EPSILON_ENV: &str = "VDBSCAN_KITTI_EPSILON";
pub const KITTI_MIN_PTS_ENV: &str = "VDBSCAN_KITTI_MIN_PTS";
pub const KITTI_METHOD_ENV: &str = "VDBSCAN_KITTI_METHOD";

const KITTI_POINT_RECORD_LEN: usize = 16;
const KIDDO_BUCKET_SIZE: usize = 2048;

type BenchmarkKdTree = KiddoKdTree<f32, u64, 3, KIDDO_BUCKET_SIZE, u32>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchmarkMethod {
    Vdbscan,
    Kiddo,
    Bruteforce,
}

pub const ALL_BENCHMARK_METHODS: [BenchmarkMethod; 3] = [
    BenchmarkMethod::Vdbscan,
    BenchmarkMethod::Kiddo,
    BenchmarkMethod::Bruteforce,
];

impl BenchmarkMethod {
    pub fn slug(self) -> &'static str {
        match self {
            Self::Vdbscan => "vdbscan",
            Self::Kiddo => "kiddo",
            Self::Bruteforce => "bruteforce",
        }
    }

    pub fn cluster(self, points: &[Point3], epsilon: f32, min_pts: usize) -> Vec<ClusterLabel> {
        match self {
            Self::Vdbscan => vdbscan::dbscan(points, epsilon, min_pts),
            Self::Kiddo => kiddo_dbscan(points, epsilon, min_pts),
            Self::Bruteforce => brute_force_dbscan(points, epsilon, min_pts),
        }
    }
}

impl std::str::FromStr for BenchmarkMethod {
    type Err = String;

    fn from_str(raw: &str) -> Result<Self, Self::Err> {
        match raw {
            "vdbscan" => Ok(Self::Vdbscan),
            "kiddo" => Ok(Self::Kiddo),
            "bruteforce" => Ok(Self::Bruteforce),
            _ => Err(format!(
                "invalid benchmark method {raw:?}; expected one of: vdbscan, kiddo, bruteforce"
            )),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BenchmarkConfig {
    pub dataset_path: PathBuf,
    pub epsilon_bits: u32,
    pub min_pts: usize,
    pub method: BenchmarkMethod,
}

impl BenchmarkConfig {
    pub fn epsilon(&self) -> f32 {
        f32::from_bits(self.epsilon_bits)
    }
}

#[derive(Debug)]
pub enum ConfigError {
    MissingPath,
    InvalidEpsilon(std::num::ParseFloatError),
    InvalidMinPts(std::num::ParseIntError),
    InvalidMethod(String),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingPath => write!(f, "missing KITTI path; set {KITTI_PATH_ENV}"),
            Self::InvalidEpsilon(err) => write!(f, "invalid epsilon value: {err}"),
            Self::InvalidMinPts(err) => write!(f, "invalid min_pts value: {err}"),
            Self::InvalidMethod(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for ConfigError {}

pub fn benchmark_config_from_env() -> Result<BenchmarkConfig, ConfigError> {
    let dataset_path = std::env::var_os(KITTI_PATH_ENV)
        .map(PathBuf::from)
        .ok_or(ConfigError::MissingPath)?;
    let epsilon = std::env::var(KITTI_EPSILON_ENV)
        .ok()
        .map(|raw| raw.parse::<f32>().map_err(ConfigError::InvalidEpsilon))
        .transpose()?
        .unwrap_or(DEFAULT_EPSILON);
    let min_pts = std::env::var(KITTI_MIN_PTS_ENV)
        .ok()
        .map(|raw| raw.parse::<usize>().map_err(ConfigError::InvalidMinPts))
        .transpose()?
        .unwrap_or(DEFAULT_MIN_PTS);
    let method = std::env::var(KITTI_METHOD_ENV)
        .ok()
        .map(|raw| {
            raw.parse::<BenchmarkMethod>()
                .map_err(ConfigError::InvalidMethod)
        })
        .transpose()?
        .unwrap_or(BenchmarkMethod::Vdbscan);
    Ok(BenchmarkConfig {
        dataset_path,
        epsilon_bits: epsilon.to_bits(),
        min_pts,
        method,
    })
}

pub fn discover_scan_paths(path: &Path) -> io::Result<Vec<PathBuf>> {
    if path.is_file() {
        return ensure_bin_extension(path).map(|_| vec![path.to_path_buf()]);
    }

    if !path.is_dir() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("path does not exist: {}", path.display()),
        ));
    }

    let mut scans = fs::read_dir(path)?
        .map(|entry| entry.map(|entry| entry.path()))
        .collect::<io::Result<Vec<_>>>()?;
    scans.retain(|entry| entry.extension().is_some_and(|ext| ext == "bin"));
    scans.sort();

    if scans.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("no .bin scans found in {}", path.display()),
        ));
    }

    Ok(scans)
}

pub fn load_kitti_scan(path: &Path) -> io::Result<Vec<Point3>> {
    let bytes = fs::read(path)?;
    parse_kitti_scan(&bytes)
}

pub fn parse_kitti_scan(bytes: &[u8]) -> io::Result<Vec<Point3>> {
    if !bytes.len().is_multiple_of(KITTI_POINT_RECORD_LEN) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "KITTI scan byte length {} is not divisible by {KITTI_POINT_RECORD_LEN}",
                bytes.len()
            ),
        ));
    }

    let mut points = Vec::with_capacity(bytes.len() / KITTI_POINT_RECORD_LEN);
    for chunk in bytes.chunks_exact(KITTI_POINT_RECORD_LEN) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
        points.push(Point3 { x, y, z });
    }

    Ok(points)
}

fn ensure_bin_extension(path: &Path) -> io::Result<()> {
    if path.extension().is_some_and(|ext| ext == "bin") {
        return Ok(());
    }

    Err(io::Error::new(
        io::ErrorKind::InvalidInput,
        format!("expected a .bin file, got {}", path.display()),
    ))
}

fn brute_force_dbscan(points: &[Point3], epsilon: f32, min_pts: usize) -> Vec<ClusterLabel> {
    assert!(epsilon > 0.0, "epsilon must be positive, got {epsilon}");

    let epsilon_sq = epsilon * epsilon;
    dbscan_with_region_query(points, min_pts, |point_idx| {
        let point = points[point_idx];
        let mut neighbors = Vec::with_capacity(min_pts);
        for (candidate_idx, candidate) in points.iter().enumerate() {
            if candidate_idx != point_idx && point.distance_sq(*candidate) <= epsilon_sq {
                neighbors.push(candidate_idx);
            }
        }
        neighbors
    })
}

fn kiddo_dbscan(points: &[Point3], epsilon: f32, min_pts: usize) -> Vec<ClusterLabel> {
    assert!(epsilon > 0.0, "epsilon must be positive, got {epsilon}");

    let epsilon_sq = epsilon * epsilon;
    // KITTI scans can contain long runs of points with the same coordinate on a split axis.
    // The default kiddo alias uses a bucket size of 32, which panics on those distributions.
    let mut tree: BenchmarkKdTree = BenchmarkKdTree::with_capacity(points.len());
    for (idx, point) in points.iter().enumerate() {
        tree.add(&[point.x, point.y, point.z], idx as u64);
    }

    dbscan_with_region_query(points, min_pts, |point_idx| {
        let point = points[point_idx];
        tree.within_unsorted::<SquaredEuclidean>(&[point.x, point.y, point.z], epsilon_sq)
            .into_iter()
            .filter_map(|neighbor| {
                let idx = neighbor.item as usize;
                (idx != point_idx).then_some(idx)
            })
            .collect()
    })
}

fn dbscan_with_region_query<F>(
    points: &[Point3],
    min_pts: usize,
    mut region_query: F,
) -> Vec<ClusterLabel>
where
    F: FnMut(usize) -> Vec<usize>,
{
    let n = points.len();
    let mut labels: Vec<ClusterLabel> = vec![None; n];
    let mut visited = vec![false; n];
    let mut is_core = vec![false; n];
    let mut neighbors_cache: Vec<Vec<usize>> = Vec::with_capacity(n);
    let mut core_count = 0usize;

    for i in 0..n {
        let neighbors = region_query(i);
        is_core[i] = neighbors.len() >= min_pts;
        if is_core[i] {
            core_count += 1;
        }
        neighbors_cache.push(neighbors);
    }

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
            for &neighbor in &neighbors_cache[cur] {
                if visited[neighbor] {
                    continue;
                }

                visited[neighbor] = true;
                labels[neighbor] = Some(cluster);
                if is_core[neighbor] {
                    queue.push_back(neighbor);
                }
            }
        }
    }

    labels
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!("vdbscan-kitti-bench-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).unwrap();
        path
    }

    fn point_bytes(x: f32, y: f32, z: f32, intensity: f32) -> [u8; KITTI_POINT_RECORD_LEN] {
        let mut out = [0u8; KITTI_POINT_RECORD_LEN];
        out[0..4].copy_from_slice(&x.to_le_bytes());
        out[4..8].copy_from_slice(&y.to_le_bytes());
        out[8..12].copy_from_slice(&z.to_le_bytes());
        out[12..16].copy_from_slice(&intensity.to_le_bytes());
        out
    }

    #[test]
    fn parse_kitti_scan_rejects_invalid_record_length() {
        let err = parse_kitti_scan(&[0u8; 15]).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidData);
    }

    #[test]
    fn parse_kitti_scan_maps_xyz_values() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&point_bytes(1.0, 2.0, 3.0, 0.5));
        bytes.extend_from_slice(&point_bytes(-4.0, 5.5, 6.25, 0.9));

        let points = parse_kitti_scan(&bytes).unwrap();

        assert_eq!(
            points,
            vec![
                Point3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0
                },
                Point3 {
                    x: -4.0,
                    y: 5.5,
                    z: 6.25
                }
            ]
        );
    }

    #[test]
    fn discover_scan_paths_supports_single_file_and_directory() {
        let dir = temp_dir("discover");
        let scan_a = dir.join("000000.bin");
        let scan_b = dir.join("000001.bin");
        let note = dir.join("readme.txt");
        fs::write(&scan_a, point_bytes(0.0, 0.0, 0.0, 0.0)).unwrap();
        fs::write(&scan_b, point_bytes(1.0, 1.0, 1.0, 0.0)).unwrap();
        fs::write(&note, b"ignore").unwrap();

        let from_file = discover_scan_paths(&scan_b).unwrap();
        let from_dir = discover_scan_paths(&dir).unwrap();

        assert_eq!(from_file, vec![scan_b.clone()]);
        assert_eq!(from_dir, vec![scan_a, scan_b]);

        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn benchmark_config_reads_env_values() {
        unsafe {
            std::env::set_var(KITTI_PATH_ENV, "/tmp/scan.bin");
            std::env::set_var(KITTI_EPSILON_ENV, "0.75");
            std::env::set_var(KITTI_MIN_PTS_ENV, "9");
            std::env::set_var(KITTI_METHOD_ENV, "kiddo");
        }

        let config = benchmark_config_from_env().unwrap();

        assert_eq!(config.dataset_path, PathBuf::from("/tmp/scan.bin"));
        assert_eq!(config.epsilon(), 0.75);
        assert_eq!(config.min_pts, 9);
        assert_eq!(config.method, BenchmarkMethod::Kiddo);

        unsafe {
            std::env::remove_var(KITTI_PATH_ENV);
            std::env::remove_var(KITTI_EPSILON_ENV);
            std::env::remove_var(KITTI_MIN_PTS_ENV);
            std::env::remove_var(KITTI_METHOD_ENV);
        }
    }

    #[test]
    fn all_benchmark_methods_match_on_small_fixture() {
        let points = vec![
            Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point3 {
                x: 0.1,
                y: 0.1,
                z: 0.0,
            },
            Point3 {
                x: 5.0,
                y: 5.0,
                z: 5.0,
            },
            Point3 {
                x: 5.1,
                y: 5.0,
                z: 5.0,
            },
            Point3 {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            },
        ];

        let expected = BenchmarkMethod::Vdbscan.cluster(&points, 0.25, 1);
        for method in ALL_BENCHMARK_METHODS {
            assert_eq!(method.cluster(&points, 0.25, 1), expected, "{method:?}");
        }
    }

    #[test]
    fn kiddo_backend_handles_many_points_with_same_axis_value() {
        let points = (0..128)
            .map(|i| Point3 {
                x: 0.0,
                y: i as f32 * 0.01,
                z: 0.0,
            })
            .collect::<Vec<_>>();

        let labels = BenchmarkMethod::Kiddo.cluster(&points, 0.02, 1);

        assert_eq!(labels.len(), points.len());
        assert!(labels.iter().any(Option::is_some));
    }
}
