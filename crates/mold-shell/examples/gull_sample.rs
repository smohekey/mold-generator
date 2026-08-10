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

    let lower_region = ManifoldSolid(chord_region(&spec, -500.0, 0.0, 80.0)?);
    let upper_region = ManifoldSolid(chord_region(&spec, 0.0, 500.0, 80.0)?);
    let lower_flange = ManifoldSolid(chord_region(&spec, -3.0, 0.0, 12.0)?);
    let upper_flange = ManifoldSolid(chord_region(&spec, 0.0, 3.0, 12.0)?);

    // The exact same solids are exported as loose registration inserts and
    // used as boolean cutters in both mold halves. This deliberately has zero
    // clearance for visual validation; production clearance can be added once
    // the socket placement is confirmed.
    let insert_a = ManifoldSolid(diamond_prism(
        &spec,
        FlangeSide::Leading,
        95.0,
        5.0,
        7.0,
        -2.0,
        2.0,
    )?);
    let insert_b = ManifoldSolid(diamond_prism(
        &spec,
        FlangeSide::Trailing,
        410.0,
        4.5,
        6.0,
        -2.25,
        2.25,
    )?);

    let kernel = ManifoldKernel;
    let part = ManifoldSolid(wing);
    let socket_cutters = [&insert_a, &insert_b];

    let mold = generate_sectioned_shell_mold_with_parting(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
            negative_flange: Some(&lower_flange),
            positive_flange: Some(&upper_flange),
            negative_sockets: &socket_cutters,
            positive_sockets: &socket_cutters,
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
    kernel.export_stl(&insert_a, output.join("registration-insert-a.stl"))?;
    kernel.export_stl(&insert_b, output.join("registration-insert-b.stl"))?;

    let mut assembly = Vec::with_capacity(3 + mold.negative.len() + mold.positive.len());
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
    assembly.push(ThreeMfObject {
        name: "registration-insert-a".to_owned(),
        solid: &insert_a,
    });
    assembly.push(ThreeMfObject {
        name: "registration-insert-b".to_owned(),
        solid: &insert_b,
    });

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

fn diamond_prism(
    spec: &WingSpec,
    side: FlangeSide,
    center_span: f64,
    chord_half_width: f64,
    span_half_width: f64,
    normal_min: f64,
    normal_max: f64,
) -> Result<Manifold, Box<dyn std::error::Error>> {
    let center = interpolate_station(spec, center_span)?;
    let inboard = interpolate_station(spec, center_span - span_half_width)?;
    let outboard = interpolate_station(spec, center_span + span_half_width)?;

    let center_x = flange_center_x(&center, side);
    let inboard_x = flange_center_x(&inboard, side);
    let outboard_x = flange_center_x(&outboard, side);

    let footprint = [
        transform_station(&center, center_x - chord_half_width, 0.0),
        transform_station(&inboard, inboard_x, 0.0),
        transform_station(&center, center_x + chord_half_width, 0.0),
        transform_station(&outboard, outboard_x, 0.0),
    ];
    let normal = flange_normal(spec, side, center_span)?;
    let base = footprint.map(|point| add_scaled(point, normal, normal_min));
    let top = footprint.map(|point| add_scaled(point, normal, normal_max));

    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };
    for point in base.into_iter().chain(top) {
        mesh.vert_properties.extend(point);
    }

    connect_ring(&mut mesh, 0, 4);
    mesh.tri_verts.extend([0, 1, 2, 0, 2, 3]);
    mesh.tri_verts.extend([4, 6, 5, 4, 7, 6]);

    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(format!("invalid diamond prism: {}", solid.status()).into());
    }
    Ok(solid)
}

fn flange_normal(
    spec: &WingSpec,
    side: FlangeSide,
    span: f64,
) -> Result<[f64; 3], Box<dyn std::error::Error>> {
    let center = interpolate_station(spec, span)?;
    let inboard = interpolate_station(spec, span - 1.0)?;
    let outboard = interpolate_station(spec, span + 1.0)?;
    let center_x = flange_center_x(&center, side);

    let chord_a = transform_station(&center, center_x - 1.0, 0.0);
    let chord_b = transform_station(&center, center_x + 1.0, 0.0);
    let span_a = transform_station(&inboard, flange_center_x(&inboard, side), 0.0);
    let span_b = transform_station(&outboard, flange_center_x(&outboard, side), 0.0);

    let chord_tangent = sub(chord_b, chord_a);
    let span_tangent = sub(span_b, span_a);
    let mut normal = normalize(cross(chord_tangent, span_tangent))?;

    if normal[2] < 0.0 {
        normal = [-normal[0], -normal[1], -normal[2]];
    }
    Ok(normal)
}

fn flange_center_x(station: &WingStation, side: FlangeSide) -> f64 {
    match side {
        FlangeSide::Leading => -6.0,
        FlangeSide::Trailing => station.chord + 6.0,
    }
}

fn connect_ring(mesh: &mut MeshGL64, lower: u64, upper: u64) {
    for i in 0..4_u64 {
        let next = (i + 1) % 4;
        mesh.tri_verts.extend([lower + i, upper + i, upper + next]);
        mesh.tri_verts
            .extend([lower + i, upper + next, lower + next]);
    }
}

fn add_scaled(point: [f64; 3], direction: [f64; 3], scale: f64) -> [f64; 3] {
    [
        point[0] + direction[0] * scale,
        point[1] + direction[1] * scale,
        point[2] + direction[2] * scale,
    ]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f64; 3]) -> Result<[f64; 3], Box<dyn std::error::Error>> {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length <= f64::EPSILON {
        return Err("cannot determine flange normal".into());
    }
    Ok([v[0] / length, v[1] / length, v[2] / length])
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
