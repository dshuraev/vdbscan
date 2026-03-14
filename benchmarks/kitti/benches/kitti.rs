use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use vdbscan::dbscan;
use vdbscan_kitti_bench::{benchmark_config_from_env, discover_scan_paths, load_kitti_scan};

fn criterion_benchmark(c: &mut Criterion) {
    let config = benchmark_config_from_env().unwrap_or_else(|err| panic!("{err}"));

    let scan_paths = discover_scan_paths(&config.dataset_path).unwrap_or_else(|err| {
        panic!(
            "failed to discover KITTI scans from {}: {err}",
            config.dataset_path.display()
        )
    });

    let mut group = c.benchmark_group("kitti_dbscan");
    group.sample_size(10);

    for scan_path in scan_paths {
        let points = load_kitti_scan(&scan_path).unwrap_or_else(|err| {
            panic!("failed to load KITTI scan {}: {err}", scan_path.display())
        });

        group.throughput(Throughput::Elements(points.len() as u64));
        group.bench_with_input(
            BenchmarkId::from_parameter(scan_path.display().to_string()),
            &points,
            |b, points| {
                b.iter(|| dbscan(points, config.epsilon(), config.min_pts));
            },
        );
    }

    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
