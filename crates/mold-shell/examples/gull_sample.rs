use std::{fs, num::NonZeroUsize, path::Path};

use manifold_rust::{manifold::Manifold, types::MeshGL64};
use mold_core::Axis;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{
    PartingRegions, ShellSettings, generate_sectioned_shell_mold_with_parting,
};
use mold_test_models::{WingSpec, WingStation};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = Path::new("target/sample-mold");
    fs::create_dir_all(output)?;

    let mut spec = mold_test_models::preset("gull")?;
    // Keep the visual sample reasonably quick to generate while preserving
    // the gull, taper, sweep, dihedral and twist characteristics.
    spec.profile_points = 24;

    let wing = mold_test_models::generate(&spec)?;
    mold_test_models::write_stl(&wing, output.join("gull-wing.stl"))?;

    let lower_region = chord_region(&spec, -500.0, 0.0, 80.0)?;
    let upper_region = chord_region(&spec, 0.0, 500.0, 80.0)?;

    let kernel = ManifoldKernel;
    let part = ManifoldSolid(wing);
    let lower_region = ManifoldSolid(lower_region);
    let upper_region = ManifoldSolid(upper_region);
    let mold = generate_sectioned_shell_mold_with_parting(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
        },
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

/// Build a closed clipping volume whose inner boundary follows each wing
/// station's local chord plane. `z_min`/`z_max` are local airfoil coordinates,
/// so twist is respected as well as the station's gull/dihedral offset.
fn chord_region(
    spec: &WingSpec,
    z_min: f64,
    z_max: f64,
    chord_margin: f64,
) -> Result<Manifold, Box<dyn std::error::Error>> {
    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };

    for station in &spec.stations {
        let x_min = -chord_margin;
        let x_max = station.chord + chord_margin;
        for &(x, z) in &[
            (x_min, z_min),
            (x_max, z_min),
            (x_max, z_max),
            (x_min, z_max),
        ] {
            let [x, y, z] = transform_station(station, x, z);
            mesh.vert_properties.extend([x, y, z]);
        }
    }

    let stations = spec.stations.len();
    for s in 0..stations - 1 {
        let a = (s * 4) as u64;
        let b = ((s + 1) * 4) as u64;
        for i in 0..4_u64 {
            let j = (i + 1) % 4;
            mesh.tri_verts.extend([a + i, b + i, b + j]);
            mesh.tri_verts.extend([a + i, b + j, a + j]);
        }
    }

    // Root and tip caps. The rectangle ordering is counter-clockwise when
    // viewed from -Y at the root, matching the side-face winding above.
    mesh.tri_verts.extend([0, 1, 2, 0, 2, 3]);
    let end = ((stations - 1) * 4) as u64;
    mesh.tri_verts
        .extend([end, end + 2, end + 1, end, end + 3, end + 2]);

    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(format!("invalid chord parting region: {}", solid.status()).into());
    }
    Ok(solid)
}

fn transform_station(station: &WingStation, x: f64, z: f64) -> [f64; 3] {
    let pivot = 0.25 * station.chord;
    let a = station.twist_deg.to_radians();
    let dx = x - pivot;
    [
        pivot + dx * a.cos() + z * a.sin() + station.x_offset,
        station.span,
        -dx * a.sin() + z * a.cos() + station.z_offset,
    ]
}
