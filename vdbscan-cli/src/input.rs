use anyhow::{Context, Result, anyhow, bail};
use std::path::Path;
use vdbscan::PointCloud;

pub fn read_kitti(path: &Path) -> Result<PointCloud> {
    let data = std::fs::read(path)?;
    if data.len() % 16 != 0 {
        bail!(
            "file size {} is not a multiple of 16 (expected 4×f32 per point: x, y, z, intensity)",
            data.len()
        );
    }
    let n = data.len() / 16;
    let mut cloud = PointCloud::with_capacity(n);
    for chunk in data.chunks_exact(16) {
        let x = f32::from_le_bytes(chunk[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(chunk[4..8].try_into().unwrap());
        let z = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
        // intensity at bytes [12..16] is intentionally ignored
        cloud.push(x, y, z);
    }
    Ok(cloud)
}

pub fn read_csv(path: &Path) -> Result<PointCloud> {
    let text = std::fs::read_to_string(path)?;
    let mut cloud = PointCloud::new();

    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        // Split on comma or whitespace depending on content
        let parts: Vec<&str> = if line.contains(',') {
            line.split(',').map(str::trim).collect()
        } else {
            line.split_ascii_whitespace().collect()
        };

        if parts.len() < 3 {
            bail!(
                "line {}: expected at least 3 columns (x, y, z), got {}",
                line_no + 1,
                parts.len()
            );
        }

        // If the first cell of the first line is not a number, treat the line as a header
        let x = match parts[0].parse::<f32>() {
            Ok(v) => v,
            Err(_) if line_no == 0 => continue,
            Err(e) => bail!("line {}: cannot parse x '{}': {}", line_no + 1, parts[0], e),
        };
        let y = parts[1]
            .parse::<f32>()
            .with_context(|| format!("line {}: cannot parse y '{}'", line_no + 1, parts[1]))?;
        let z = parts[2]
            .parse::<f32>()
            .with_context(|| format!("line {}: cannot parse z '{}'", line_no + 1, parts[2]))?;

        cloud.push(x, y, z);
    }

    Ok(cloud)
}

// ---------------------------------------------------------------------------
// PLY reader
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum PlyType {
    Char,
    Uchar,
    Short,
    Ushort,
    Int,
    Uint,
    Float,
    Double,
}

impl PlyType {
    fn from_str(s: &str) -> Result<Self> {
        match s {
            "char" | "int8" => Ok(PlyType::Char),
            "uchar" | "uint8" => Ok(PlyType::Uchar),
            "short" | "int16" => Ok(PlyType::Short),
            "ushort" | "uint16" => Ok(PlyType::Ushort),
            "int" | "int32" => Ok(PlyType::Int),
            "uint" | "uint32" => Ok(PlyType::Uint),
            "float" | "float32" => Ok(PlyType::Float),
            "double" | "float64" => Ok(PlyType::Double),
            _ => bail!("unsupported PLY property type '{}'", s),
        }
    }

    fn byte_size(self) -> usize {
        match self {
            PlyType::Char | PlyType::Uchar => 1,
            PlyType::Short | PlyType::Ushort => 2,
            PlyType::Int | PlyType::Uint | PlyType::Float => 4,
            PlyType::Double => 8,
        }
    }

    fn read_le_f32(self, buf: &[u8]) -> f32 {
        match self {
            PlyType::Float => f32::from_le_bytes(buf[..4].try_into().unwrap()),
            PlyType::Double => f64::from_le_bytes(buf[..8].try_into().unwrap()) as f32,
            PlyType::Int => i32::from_le_bytes(buf[..4].try_into().unwrap()) as f32,
            PlyType::Uint => u32::from_le_bytes(buf[..4].try_into().unwrap()) as f32,
            PlyType::Short => i16::from_le_bytes(buf[..2].try_into().unwrap()) as f32,
            PlyType::Ushort => u16::from_le_bytes(buf[..2].try_into().unwrap()) as f32,
            PlyType::Char => buf[0] as i8 as f32,
            PlyType::Uchar => buf[0] as f32,
        }
    }

    fn read_be_f32(self, buf: &[u8]) -> f32 {
        match self {
            PlyType::Float => f32::from_be_bytes(buf[..4].try_into().unwrap()),
            PlyType::Double => f64::from_be_bytes(buf[..8].try_into().unwrap()) as f32,
            PlyType::Int => i32::from_be_bytes(buf[..4].try_into().unwrap()) as f32,
            PlyType::Uint => u32::from_be_bytes(buf[..4].try_into().unwrap()) as f32,
            PlyType::Short => i16::from_be_bytes(buf[..2].try_into().unwrap()) as f32,
            PlyType::Ushort => u16::from_be_bytes(buf[..2].try_into().unwrap()) as f32,
            PlyType::Char => buf[0] as i8 as f32,
            PlyType::Uchar => buf[0] as f32,
        }
    }
}

struct PlyProp {
    name: String,
    ty: PlyType,
}

enum PlyFormat {
    Ascii,
    BinaryLe,
    BinaryBe,
}

pub fn read_ply(path: &Path) -> Result<PointCloud> {
    let raw = std::fs::read(path)?;

    let (header_end, data_start) =
        find_end_header(&raw).ok_or_else(|| anyhow!("PLY: missing 'end_header'"))?;

    let header_str =
        std::str::from_utf8(&raw[..header_end]).context("PLY: header contains non-UTF-8 bytes")?;
    let data = &raw[data_start..];

    let mut format = PlyFormat::Ascii;
    let mut vertex_count = 0usize;
    let mut props: Vec<PlyProp> = Vec::new();
    let mut in_vertex = false;
    let mut found_vertex = false;

    for (i, line) in header_str.lines().enumerate() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.as_slice() {
            ["ply"] | ["comment", ..] | ["obj_info", ..] => {}
            ["format", fmt, _ver] => {
                format = match *fmt {
                    "ascii" => PlyFormat::Ascii,
                    "binary_little_endian" => PlyFormat::BinaryLe,
                    "binary_big_endian" => PlyFormat::BinaryBe,
                    _ => bail!("PLY: unsupported format '{}'", fmt),
                };
            }
            ["element", "vertex", n] => {
                vertex_count = n
                    .parse()
                    .with_context(|| format!("PLY line {}: invalid vertex count", i + 1))?;
                in_vertex = true;
                found_vertex = true;
                props.clear();
            }
            ["element", ..] => {
                in_vertex = false;
            }
            ["property", "list", ..] if in_vertex => {
                bail!("PLY: list properties in the vertex element are not supported");
            }
            ["property", ty, name] if in_vertex => {
                props.push(PlyProp { name: name.to_string(), ty: PlyType::from_str(ty)? });
            }
            _ => {}
        }
    }

    if !found_vertex {
        bail!("PLY: no 'element vertex' found in header");
    }

    let x_idx = props
        .iter()
        .position(|p| p.name == "x")
        .ok_or_else(|| anyhow!("PLY: vertex element has no 'x' property"))?;
    let y_idx = props
        .iter()
        .position(|p| p.name == "y")
        .ok_or_else(|| anyhow!("PLY: vertex element has no 'y' property"))?;
    let z_idx = props
        .iter()
        .position(|p| p.name == "z")
        .ok_or_else(|| anyhow!("PLY: vertex element has no 'z' property"))?;

    let mut cloud = PointCloud::with_capacity(vertex_count);

    match format {
        PlyFormat::Ascii => {
            let text =
                std::str::from_utf8(data).context("PLY: ASCII data contains non-UTF-8 bytes")?;
            let mut lines = text.lines().filter(|l| !l.trim().is_empty());
            for v in 0..vertex_count {
                let line = lines.next().ok_or_else(|| {
                    anyhow!("PLY: unexpected end of data at vertex {} (expected {})", v, vertex_count)
                })?;
                let cols: Vec<&str> = line.split_whitespace().collect();
                if cols.len() < props.len() {
                    bail!(
                        "PLY: vertex {} has {} columns, expected {}",
                        v,
                        cols.len(),
                        props.len()
                    );
                }
                let x = cols[x_idx]
                    .parse::<f32>()
                    .with_context(|| format!("PLY: vertex {}: cannot parse x '{}'", v, cols[x_idx]))?;
                let y = cols[y_idx]
                    .parse::<f32>()
                    .with_context(|| format!("PLY: vertex {}: cannot parse y '{}'", v, cols[y_idx]))?;
                let z = cols[z_idx]
                    .parse::<f32>()
                    .with_context(|| format!("PLY: vertex {}: cannot parse z '{}'", v, cols[z_idx]))?;
                cloud.push(x, y, z);
            }
        }
        PlyFormat::BinaryLe => {
            let stride: usize = props.iter().map(|p| p.ty.byte_size()).sum();
            let x_off: usize = props[..x_idx].iter().map(|p| p.ty.byte_size()).sum();
            let y_off: usize = props[..y_idx].iter().map(|p| p.ty.byte_size()).sum();
            let z_off: usize = props[..z_idx].iter().map(|p| p.ty.byte_size()).sum();
            if data.len() < stride * vertex_count {
                bail!(
                    "PLY: binary data too short: {} bytes available, {} required \
                     (stride={}, vertices={})",
                    data.len(),
                    stride * vertex_count,
                    stride,
                    vertex_count
                );
            }
            for i in 0..vertex_count {
                let row = &data[i * stride..];
                cloud.push(
                    props[x_idx].ty.read_le_f32(&row[x_off..]),
                    props[y_idx].ty.read_le_f32(&row[y_off..]),
                    props[z_idx].ty.read_le_f32(&row[z_off..]),
                );
            }
        }
        PlyFormat::BinaryBe => {
            let stride: usize = props.iter().map(|p| p.ty.byte_size()).sum();
            let x_off: usize = props[..x_idx].iter().map(|p| p.ty.byte_size()).sum();
            let y_off: usize = props[..y_idx].iter().map(|p| p.ty.byte_size()).sum();
            let z_off: usize = props[..z_idx].iter().map(|p| p.ty.byte_size()).sum();
            if data.len() < stride * vertex_count {
                bail!(
                    "PLY: binary data too short: {} bytes available, {} required \
                     (stride={}, vertices={})",
                    data.len(),
                    stride * vertex_count,
                    stride,
                    vertex_count
                );
            }
            for i in 0..vertex_count {
                let row = &data[i * stride..];
                cloud.push(
                    props[x_idx].ty.read_be_f32(&row[x_off..]),
                    props[y_idx].ty.read_be_f32(&row[y_off..]),
                    props[z_idx].ty.read_be_f32(&row[z_off..]),
                );
            }
        }
    }

    Ok(cloud)
}

/// Returns `(header_text_end, data_start)`.
/// `raw[..header_text_end]` is the header text (everything before "end_header").
/// `raw[data_start..]` is the payload (everything after the "end_header\n" line).
fn find_end_header(raw: &[u8]) -> Option<(usize, usize)> {
    const NEEDLE: &[u8] = b"end_header";
    for i in 0..raw.len().saturating_sub(NEEDLE.len()) {
        if raw[i..].starts_with(NEEDLE) {
            let after = i + NEEDLE.len();
            if raw.get(after) == Some(&b'\n') {
                return Some((i, after + 1));
            }
            if raw.get(after) == Some(&b'\r') && raw.get(after + 1) == Some(&b'\n') {
                return Some((i, after + 2));
            }
        }
    }
    None
}
