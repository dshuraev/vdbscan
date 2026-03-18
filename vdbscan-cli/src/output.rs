use anyhow::{bail, Result};
use std::collections::HashMap;
use std::io::{BufWriter, Write};
use std::path::Path;
use vdbscan::Clustering;

use crate::color::label_to_rgb;

/// Write clustering as binary little-endian PLY with per-point label and RGB color.
pub fn write_ply(path: &Path, clustering: &Clustering) -> Result<()> {
    let file = std::fs::File::create(path)?;
    let mut w = BufWriter::new(file);

    let n = clustering.len();
    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "element vertex {}", n)?;
    writeln!(w, "property float x")?;
    writeln!(w, "property float y")?;
    writeln!(w, "property float z")?;
    writeln!(w, "property int label")?;
    writeln!(w, "property uchar red")?;
    writeln!(w, "property uchar green")?;
    writeln!(w, "property uchar blue")?;
    writeln!(w, "end_header")?;

    for (pt, label) in clustering.iter() {
        let label_i32: i32 = match label {
            None => -1,
            Some(id) => id.get() as i32,
        };
        let [r, g, b] = label_to_rgb(label);
        w.write_all(&pt.x.to_le_bytes())?;
        w.write_all(&pt.y.to_le_bytes())?;
        w.write_all(&pt.z.to_le_bytes())?;
        w.write_all(&label_i32.to_le_bytes())?;
        w.write_all(&[r, g, b])?;
    }

    Ok(())
}

/// Write clustering as multiple XYZ files into `dir`: `noise.xyz` and `cluster_N.xyz` (1-based).
pub fn write_xyz_multi(dir: &Path, clustering: &Clustering) -> Result<()> {
    if dir.exists() && !dir.is_dir() {
        bail!("'{}' exists and is not a directory", dir.display());
    }
    std::fs::create_dir_all(dir)?;

    // Bucket points by label.
    let mut noise: Vec<(f32, f32, f32)> = Vec::new();
    let mut clusters: HashMap<usize, Vec<(f32, f32, f32)>> = HashMap::new();

    for (pt, label) in clustering.iter() {
        match label {
            None => noise.push((pt.x, pt.y, pt.z)),
            Some(id) => clusters.entry(id.get()).or_default().push((pt.x, pt.y, pt.z)),
        }
    }

    // Write noise.xyz (always, even if empty).
    write_xyz_file(&dir.join("noise.xyz"), &noise)?;

    // Write cluster_N.xyz in sorted order.
    let mut ids: Vec<usize> = clusters.keys().copied().collect();
    ids.sort_unstable();
    for id in ids {
        write_xyz_file(&dir.join(format!("cluster_{}.xyz", id)), &clusters[&id])?;
    }

    Ok(())
}

fn write_xyz_file(path: &Path, points: &[(f32, f32, f32)]) -> Result<()> {
    let file = std::fs::File::create(path)?;
    let mut w = BufWriter::new(file);
    for &(x, y, z) in points {
        writeln!(w, "{} {} {}", x, y, z)?;
    }
    Ok(())
}

/// Write clustering as CSV with columns: x, y, z, label (-1 = noise).
pub fn write_csv(path: &Path, clustering: &Clustering) -> Result<()> {
    let file = std::fs::File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "x,y,z,label")?;
    for (pt, label) in clustering.iter() {
        let label_i32: i32 = match label {
            None => -1,
            Some(id) => id.get() as i32,
        };
        writeln!(w, "{},{},{},{}", pt.x, pt.y, pt.z, label_i32)?;
    }

    Ok(())
}
