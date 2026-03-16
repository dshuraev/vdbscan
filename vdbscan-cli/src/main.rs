mod color;
mod input;
mod output;

use anyhow::{Context, Result, bail};
use clap::Parser;
use std::path::{Path, PathBuf};
use vdbscan::dbscan;

#[derive(Parser)]
#[command(name = "vdbscan", about = "Density-based point cloud clustering")]
struct Args {
    /// Path to input point cloud file
    #[arg(short = 'i', long = "input")]
    input: PathBuf,

    /// Path to output file
    #[arg(short = 'o', long = "output")]
    output: PathBuf,

    /// DBSCAN epsilon radius in metres
    #[arg(long)]
    epsilon: f32,

    /// DBSCAN minimum points for core point
    #[arg(long = "min-pts")]
    min_pts: usize,

    /// Override input format detection
    #[arg(long = "input-format")]
    input_format: Option<InputFormat>,
}

#[derive(Clone, clap::ValueEnum)]
enum InputFormat {
    Kitti,
    Ply,
    Csv,
    Xyz,
}

enum OutputFormat {
    Ply,
    Csv,
}

fn detect_input_format(path: &Path, override_fmt: Option<&InputFormat>) -> Result<InputFormat> {
    if let Some(fmt) = override_fmt {
        return Ok(fmt.clone());
    }
    match path.extension().and_then(|e| e.to_str()) {
        Some("bin") => Ok(InputFormat::Kitti),
        Some("ply") => Ok(InputFormat::Ply),
        Some("csv") => Ok(InputFormat::Csv),
        Some("xyz") => Ok(InputFormat::Xyz),
        Some(ext) => bail!(
            "unknown input extension '.{}'; supported: .bin (KITTI), .ply, .csv, .xyz — \
             use --input-format to override",
            ext
        ),
        None => bail!(
            "cannot detect input format: '{}' has no file extension; \
             supported: .bin (KITTI), .ply, .csv, .xyz — use --input-format to override",
            path.display()
        ),
    }
}

fn detect_output_format(path: &Path) -> Result<OutputFormat> {
    match path.extension().and_then(|e| e.to_str()) {
        Some("ply") => Ok(OutputFormat::Ply),
        Some("csv") => Ok(OutputFormat::Csv),
        Some(ext) => bail!("unknown output extension '.{}'; supported: .ply, .csv", ext),
        None => bail!(
            "cannot detect output format: '{}' has no file extension; supported: .ply, .csv",
            path.display()
        ),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    let input_fmt = detect_input_format(&args.input, args.input_format.as_ref())?;
    let output_fmt = detect_output_format(&args.output)?;

    let cloud = match input_fmt {
        InputFormat::Kitti => input::read_kitti(&args.input)
            .with_context(|| format!("failed to read KITTI file '{}'", args.input.display()))?,
        InputFormat::Ply => input::read_ply(&args.input)
            .with_context(|| format!("failed to read PLY file '{}'", args.input.display()))?,
        InputFormat::Csv | InputFormat::Xyz => input::read_csv(&args.input)
            .with_context(|| format!("failed to read CSV/XYZ file '{}'", args.input.display()))?,
    };

    let clustering = dbscan(&cloud, args.epsilon, args.min_pts);

    match output_fmt {
        OutputFormat::Ply => output::write_ply(&args.output, &clustering)
            .with_context(|| format!("failed to write PLY file '{}'", args.output.display()))?,
        OutputFormat::Csv => output::write_csv(&args.output, &clustering)
            .with_context(|| format!("failed to write CSV file '{}'", args.output.display()))?,
    }

    Ok(())
}
