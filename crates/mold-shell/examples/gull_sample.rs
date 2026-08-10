use std::{fs, num::NonZeroUsize, path::Path};

use manifold_rust::{manifold::Manifold, types::MeshGL64};
use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::Axis;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{PartingRegions, ShellSettings, generate_sectioned_shell_mold_with_parting};
use mold_test_models::{WingSpec, WingStation};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = Path::new("target/sample-mold");
    fs::create_dir_all(output)?;

    let mut spec = mold_test_models::preset("gull")?;
    spec.profile_points = 24;

    let wing = mold_test_models::generate(&spec)?;
    mold_test_models::write_stl(&wing, output.join("gull-wing.stl"))?;

    let lower_region = chord_region(&spec, -500.0, 0.0, 80.0)?;
    let upper_region = chord_region(&spec, 0.0, 500.0, 80.0)?;
    let lower_flange = chord_region(&spec, -3.0, 0.0, 12.0)?;
    let upper_flange = chord_region(&spec, 0.0, 3.0, 12.0)?;

    // Two deliberately asymmetric FDM-friendly registration keys. They are
    // centered in the flange strips outside the airfoil envelope rather than
    // over the wing itself. Each feature is built from interpolated local wing
    // stations, so it follows the local parting surface and twist.
    let key_a = diamond_key(&spec, FlangeSide::Leading, 95.0, 14.0, 4.5, 3.0)?;
    let key_b = diamond_key(&spec, FlangeSide::Trailing, 410.0, 12.0, 4.0, 3.5)?;
    let socket_a = diamond_key(&spec, FlangeSide::Leading, 95.0, 14.6, 4.8, 3.25)?;
    let socket_b = diamond_key(&spec, FlangeSide::Trailing, 410.0, 12.6, 4.3, 3.75)?;

    let kernel = ManifoldKernel;
    let part = ManifoldSolid(wing);
    let lower_region = ManifoldSolid(lower_region);
    let upper_region = ManifoldSolid(upper_region);
    let lower_flange = ManifoldSolid(lower_flange);
    let upper_flange = ManifoldSolid(upper_flange);
    let key_a = ManifoldSolid(key_a);
    let key_b = ManifoldSolid(key_b);
    let socket_a = ManifoldSolid(socket_a);
    let socket_b = ManifoldSolid(socket_b);
    let keys = [&key_a, &key_b];
    let sockets = [&socket_a, &socket_b];

    let mold = generate_sectioned_shell_mold_with_parting(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
            negative_flange: Some(&lower_flange),
            positive_flange: Some(&upper_flange),
            negative_keys: &keys,
            positive_sockets: &sockets,
        },
        Axis::Y,
        NonZeroUsize::new(2).unwrap(),
        ShellSettings::default(),
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

    let mut assembly = Vec::with_capacity(1 + mold.negative.len() + mold.positive.len());
    assembly.push(ThreeMfObject {
        name: "wing".to_owned(),
        solid: &part,
    });
    for (index, piece) in mold.negative.iter().enumerate() {
        assembly.push(ThreeMfObject {
            name: format!("mold-lower-{:02}", index + 1),
            solid: piece,
        });
    }
    for (index, piece) in mold.positive.iter().enumerate() {
        assembly.push(ThreeMfObject {
            name: format!("mold-upper-{:02}", index + 1),
            solid: piece,
        });
    }
    write_3mf(
        output.join("gull-wing-mold-assembly.3mf"),
        "Gull wing mold validation assembly",
        &assembly,
    )?;

    println!("generated visual sample in {}", output.display());
    Ok(())
}

#[derive(Debug, Clone, Copy)]
enum FlangeSide {
    Leading,
    Trailing,
}

fn diamond_key(
    spec: &WingSpec,
    side: FlangeSide,
    center_span: f64,
    span_width: f64,
    half_width: f64,
    height: f64,
) -> Result<Manifold, Box<dyn std::error::Error>> {
    let stations = [
        interpolate_station(spec, center_span - span_width * 0.5)?,
        interpolate_station(spec, center_span + span_width * 0.5)?,
    ];

    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };

    for station in &stations {
        // The flange extends 12 mm beyond each chord edge. Centering 6 mm
        // outside the airfoil puts the feature in the middle of that strip.
        let center_x = match side {
            FlangeSide::Leading => -6.0,
            FlangeSide::Trailing => station.chord + 6.0,
        };
        for &(x, z) in &[
            (center_x - half_width, 0.0),
            (center_x, -height),
            (center_x + half_width, 0.0),
            (center_x, height),
        ] {
            mesh.vert_properties
                .extend(transform_station(station, x, z));
        }
    }

    close_loft(&mut mesh, stations.len());
    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(format!("invalid diamond key: {}", solid.status()).into());
    }
    Ok(solid)
}

fn interpolate_station(
    spec: &WingSpec,
    span: f64,
) -> Result<WingStation, Box<dyn std::error::Error>> {
    for pair in spec.stations.windows(2) {
        let a = pair[0];
        let b = pair[1];
        if span >= a.span && span <= b.span {
            let t = (span - a.span) / (b.span - a.span);
            return Ok(WingStation {
                span,
                chord: lerp(a.chord, b.chord, t),
                x_offset: lerp(a.x_offset, b.x_offset, t),
                z_offset: lerp(a.z_offset, b.z_offset, t),
                twist_deg: lerp(a.twist_deg, b.twist_deg, t),
            });
        }
    }
    Err(format!("span {span} is outside the wing fixture").into())
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

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
        for &(x, z) in &[
            (-chord_margin, z_min),
            (station.chord + chord_margin, z_min),
            (station.chord + chord_margin, z_max),
            (-chord_margin, z_max),
        ] {
            mesh.vert_properties
                .extend(transform_station(station, x, z));
        }
    }

    close_loft(&mut mesh, spec.stations.len());
    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(format!("invalid chord region: {}", solid.status()).into());
    }
    Ok(solid)
}

fn close_loft(mesh: &mut MeshGL64, stations: usize) {
    for station in 0..stations - 1 {
        let a = (station * 4) as u64;
        let b = ((station + 1) * 4) as u64;
        for i in 0..4_u64 {
            let j = (i + 1) % 4;
            mesh.tri_verts.extend([a + i, b + i, b + j]);
            mesh.tri_verts.extend([a + i, b + j, a + j]);
        }
    }
    mesh.tri_verts.extend([0, 1, 2, 0, 2, 3]);
    let end = ((stations - 1) * 4) as u64;
    mesh.tri_verts
        .extend([end, end + 2, end + 1, end, end + 3, end + 2]);
}

fn transform_station(station: &WingStation, x: f64, z: f64) -> [f64; 3] {
    let pivot = 0.25 * station.chord;
    let angle = station.twist_deg.to_radians();
    let dx = x - pivot;
    [
        pivot + dx * angle.cos() + z * angle.sin() + station.x_offset,
        station.span,
        -dx * angle.sin() + z * angle.cos() + station.z_offset,
    ]
}
