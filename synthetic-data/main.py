from __future__ import annotations

import random
from enum import Enum
from pathlib import Path
from typing import Annotated, Optional

import numpy as np
import typer

app = typer.Typer(add_completion=False)


class Shape(str, Enum):
    sphere = "sphere"
    box = "box"
    cylinder = "cylinder"


def _sample_sphere(n: int, radius: float, rng: np.random.Generator) -> np.ndarray:
    """Uniform sampling inside a solid sphere (radius)."""
    # Marsaglia-like: sample on unit sphere then scale by cbrt(u) for volume
    directions = rng.standard_normal((n, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    r = radius * (rng.uniform(0, 1, n) ** (1 / 3))
    return directions * r[:, None]


def _sample_box(n: int, side: float, rng: np.random.Generator) -> np.ndarray:
    """Uniform sampling inside an axis-aligned cube of given side length."""
    half = side / 2
    return rng.uniform(-half, half, (n, 3))


def _sample_cylinder(n: int, diameter: float, rng: np.random.Generator) -> np.ndarray:
    """Uniform sampling inside a cylinder: base diameter = height = diameter."""
    radius = diameter / 2
    height = diameter
    # Uniform disk sampling
    r = radius * np.sqrt(rng.uniform(0, 1, n))
    theta = rng.uniform(0, 2 * np.pi, n)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = rng.uniform(-height / 2, height / 2, n)
    return np.stack([x, y, z], axis=1)


_SAMPLERS = {
    Shape.sphere: _sample_sphere,
    Shape.box: _sample_box,
    Shape.cylinder: _sample_cylinder,
}


def _place_centers(
    n: int,
    bounds: tuple[float, float, float],
    min_sep: float,
    rng: np.random.Generator,
    max_attempts: int = 10_000,
) -> list[np.ndarray]:
    """Place n cluster centers inside bounds with minimum separation."""
    bx, by, bz = bounds
    centers: list[np.ndarray] = []
    attempts = 0
    while len(centers) < n:
        if attempts >= max_attempts:
            raise RuntimeError(
                f"Could not place {n} clusters with min_separation={min_sep} "
                f"inside scene_bounds={bounds} after {max_attempts} attempts. "
                "Try reducing --n-clusters or --min-separation, or increasing --scene-bounds."
            )
        c = np.array([rng.uniform(0, bx), rng.uniform(0, by), rng.uniform(0, bz)])
        if all(np.linalg.norm(c - other) >= min_sep for other in centers):
            centers.append(c)
        attempts += 1
    return centers


@app.command()
def generate(
    n_clusters: Annotated[int, typer.Option("--n-clusters", help="Number of clusters")] = 5,
    points_per_cluster: Annotated[
        tuple[int, int],
        typer.Option("--points-per-cluster", help="Min and max points per cluster", metavar="MIN MAX"),
    ] = (100, 500),
    cluster_std: Annotated[
        float, typer.Option("--cluster-std", help="Gaussian jitter added to sampled points (sensor noise)")
    ] = 0.05,
    cluster_size: Annotated[
        float,
        typer.Option(
            "--cluster-size",
            help="Diameter for sphere/cylinder base, side length for box",
        ),
    ] = 1.0,
    min_separation: Annotated[
        float, typer.Option("--min-separation", help="Minimum distance between cluster centers")
    ] = 2.0,
    scene_bounds: Annotated[
        tuple[float, float, float],
        typer.Option("--scene-bounds", help="Scene bounding box X Y Z", metavar="X Y Z"),
    ] = (20.0, 20.0, 10.0),
    noise_points: Annotated[
        int, typer.Option("--noise-points", help="Number of random noise points")
    ] = 200,
    output: Annotated[
        Path, typer.Option("--output", help="Output .xyz file path")
    ] = Path("cloud.xyz"),
    seed: Annotated[Optional[int], typer.Option("--seed", help="RNG seed")] = None,
    shape: Annotated[
        Optional[list[Shape]],
        typer.Option("--shape", help="Cluster shape(s); can be given multiple times"),
    ] = None,
) -> None:
    """Generate synthetic LiDAR-like 3D point cloud data."""
    rng = np.random.default_rng(seed)
    py_rng = random.Random(seed)

    shapes: list[Shape] = shape if shape else [Shape.sphere]

    pmin, pmax = points_per_cluster
    if pmin > pmax:
        typer.echo(f"Error: points-per-cluster MIN ({pmin}) > MAX ({pmax})", err=True)
        raise typer.Exit(1)

    typer.echo(f"Placing {n_clusters} cluster centers...")
    try:
        centers = _place_centers(n_clusters, scene_bounds, min_separation, rng)
    except RuntimeError as exc:
        typer.echo(f"Error: {exc}", err=True)
        raise typer.Exit(1)

    all_points: list[np.ndarray] = []

    for i, center in enumerate(centers):
        chosen_shape = py_rng.choice(shapes)
        n_pts = int(rng.integers(pmin, pmax + 1))
        sampler = _SAMPLERS[chosen_shape]
        pts = sampler(n_pts, cluster_size, rng)
        if cluster_std > 0:
            pts += rng.normal(0, cluster_std, pts.shape)
        pts += center
        all_points.append(pts)
        typer.echo(f"  cluster {i+1:>3}/{n_clusters}: shape={chosen_shape.value:>8}, n={n_pts}, center=({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f})")

    if noise_points > 0:
        bx, by, bz = scene_bounds
        noise = np.stack(
            [
                rng.uniform(0, bx, noise_points),
                rng.uniform(0, by, noise_points),
                rng.uniform(0, bz, noise_points),
            ],
            axis=1,
        )
        all_points.append(noise)

    cloud = np.vstack(all_points)
    total = len(cloud)
    typer.echo(f"Total points: {total} ({total - noise_points} cluster + {noise_points} noise)")

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w") as f:
        for pt in cloud:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")

    typer.echo(f"Saved to {output}")


if __name__ == "__main__":
    app()
