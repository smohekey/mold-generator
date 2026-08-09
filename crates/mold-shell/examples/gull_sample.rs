use std::{fs, num::NonZeroUsize, path::Path};

use mold_core::Axis;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{ShellSettings, generate_sectioned_shell_mold};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = Path::new("target/sample-mold");
    fs::create_dir_all(output)?;

    let mut spec = mold_test_models::preset("gull")?;
    // Keep the visual sample reasonably quick to generate while preserving
    // the gull, taper, sweep, dihedral and twist characteristics.
    spec.profile_points = 24;

    let wing = mold_test_models::generate(&spec)?;
    mold_test_models::write_stl(&wing, output.join("gull-wing.stl"))?;

    let kernel = ManifoldKernel;
    let part = ManifoldSolid(wing);
    let mold = generate_sectioned_shell_mold(
        &kernel,
        &part,
        Axis::Z,
        Axis::Y,
        NonZeroUsize::new(2).unwrap(),
        ShellSettings {
            thickness: 3.0,
            flange_width: 12.0,
            web_thickness: 3.0,
        },
    )?;

    for (index, piece) in mold.negative.iter().enumerate() {
        kernel.export_stl(
            piece,
            output.join(format!("mold-lower-{:02}.stl", index + 1)),
        )?;
    }
    for (index, piece) in mold.positive.iter().enumerate() {
        kernel.export_stl(
            piece,
            output.join(format!("mold-upper-{:02}.stl", index + 1)),
        )?;
    }

    println!("generated visual sample in {}", output.display());
    Ok(())
}
