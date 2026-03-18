# synthetic-data

CLI tool for generating synthetic LiDAR-like 3D point cloud data as `.xyz` files.

## Install

```bash
uv sync
```

## Usage

```bash
uv run main.py [OPTIONS]
```

### Options

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--n-clusters INT` | `5` | Number of clusters |
| `--points-per-cluster MIN MAX` | `100 500` | Point count range per cluster |
| `--cluster-std FLOAT` | `0.05` | Gaussian jitter on points (sensor noise) |
| `--cluster-size FLOAT` | `1.0` | Diameter (sphere/cylinder) or side length (box) |
| `--min-separation FLOAT` | `2.0` | Minimum distance between cluster centers |
| `--scene-bounds X Y Z` | `20 20 10` | Scene bounding box dimensions |
| `--noise-points INT` | `200` | Random noise points scattered across the scene |
| `--output PATH` | `cloud.xyz` | Output `.xyz` file |
| `--seed INT` | — | RNG seed for reproducibility |
| `--shape [sphere\|box\|cylinder]` | `sphere` | Cluster shape(s); repeat to allow multiple |

`--shape` can be given multiple times — each cluster is randomly assigned one of the provided shapes.

### Cluster shapes

- **sphere** — uniform volume sampling, diameter = `--cluster-size`
- **box** — uniform volume sampling, side = `--cluster-size` (cube)
- **cylinder** — base diameter = height = `--cluster-size`, axis aligned to Z

## Examples

Simple sphere-only cloud:

```bash
uv run main.py --n-clusters 10 --seed 42 --output cloud.xyz
```

Mixed shapes, tight clusters, large scene:

```bash
uv run main.py \
  --n-clusters 8 \
  --points-per-cluster 200 800 \
  --cluster-size 1.5 \
  --cluster-std 0.02 \
  --min-separation 3.0 \
  --scene-bounds 50 50 20 \
  --noise-points 500 \
  --shape sphere --shape box --shape cylinder \
  --seed 7 \
  --output scene.xyz
```

## Output format

Plain text, one point per line:

```text
x y z
x y z
...
```

No header. Compatible with CloudCompare, Open3D, and most LiDAR processing tools.
