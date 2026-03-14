use std::hint::black_box;
use std::process::ExitCode;

use vdbscan::dbscan;
use vdbscan_kitti_bench::{benchmark_config_from_env, discover_scan_paths, load_kitti_scan};

const PROFILE_REPEATS_ENV: &str = "VDBSCAN_KITTI_PROFILE_REPEATS";
const DEFAULT_PROFILE_REPEATS: usize = 10;

fn main() -> ExitCode {
    let config = match benchmark_config_from_env() {
        Ok(config) => config,
        Err(err) => {
            eprintln!("{err}");
            return ExitCode::FAILURE;
        }
    };

    let repeats = std::env::var(PROFILE_REPEATS_ENV)
        .ok()
        .and_then(|raw| raw.parse::<usize>().ok())
        .filter(|&value| value > 0)
        .unwrap_or(DEFAULT_PROFILE_REPEATS);

    let scan_paths = match discover_scan_paths(&config.dataset_path) {
        Ok(paths) => paths,
        Err(err) => {
            eprintln!(
                "failed to discover KITTI scans from {}: {err}",
                config.dataset_path.display()
            );
            return ExitCode::FAILURE;
        }
    };

    let scans = match scan_paths
        .iter()
        .map(|path| load_kitti_scan(path).map(|points| (path, points)))
        .collect::<Result<Vec<_>, _>>()
    {
        Ok(scans) => scans,
        Err(err) => {
            eprintln!("failed to load KITTI scan: {err}");
            return ExitCode::FAILURE;
        }
    };

    for _ in 0..repeats {
        for (_, points) in &scans {
            black_box(dbscan(points, config.epsilon(), config.min_pts));
        }
    }

    ExitCode::SUCCESS
}
