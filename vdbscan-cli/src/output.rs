use anyhow::Result;
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
