use std::{fs, path::Path};

use manifold_rust::{manifold::Manifold, types::MeshGL64};
use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::Axis;
use mold_geometry::SolidKernel;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{
    PartingRegions, ShellSettings, WebbingSettings,
    generate_sectioned_shell_mold_with_parting_ranges,
};
use mold_test_models::{WingSpec, WingStation};

const SEGMENT_COUNT: usize = 2;

#[derive(Debug, Clone, Copy)]
struct FixtureSettings {
    chord_half_width: f64,
    span_half_width: f64,
    normal_half_depth: f64,
}

#[derive(Debug, Clone, Copy)]
struct RegistrationSettings {
    leading: FixtureSettings,
    trailing: FixtureSettings,
    max_spacing_ratio: f64,
    edge_margin_ratio: f64,
    minimum_per_segment: usize,
}

impl Default for RegistrationSettings {
    fn default() -> Self {
        let fixture = FixtureSettings {
            chord_half_width: 5.0,
            span_half_width: 7.0,
            normal_half_depth: 2.25,
        };
        Self {
            leading: fixture,
            trailing: fixture,
            max_spacing_ratio: 10.0,
            edge_margin_ratio: 1.5,
            minimum_per_segment: 2,
        }
    }
}

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

    let kernel = ManifoldKernel;
    let part = ManifoldSolid(wing);
    let baseline_settings = ShellSettings {
        structural_webbing: None,
        ..Default::default()
    };
    let expanded_bounds = kernel.bounds(&kernel.offset(&part, baseline_settings.thickness)?)?;
    let segment_ranges = deviation_aware_segment_ranges(
        &spec,
        SEGMENT_COUNT,
        expanded_bounds.min.y,
        expanded_bounds.max.y,
    )?;
    println!("deviation-aware segment ranges: {segment_ranges:?}");

    let registration = RegistrationSettings::default();
    let inserts = registration_inserts(&spec, &segment_ranges, registration)?;
    println!(
        "placed {} registration inserts across {} segments",
        inserts.len(),
        segment_ranges.len()
    );
    let socket_cutters: Vec<&ManifoldSolid> = inserts.iter().map(|insert| &insert.solid).collect();
    let webbing = WebbingSettings::default();
    let (lower_ribs, upper_ribs) = diagonal_ribs(
        &spec,
        &segment_ranges,
        registration,
        webbing.thickness(),
        webbing.depth,
    )?;

    let baseline = generate_sectioned_shell_mold_with_parting_ranges(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
            negative_flange: Some(&lower_flange),
            positive_flange: Some(&upper_flange),
            negative_sockets: &socket_cutters,
            positive_sockets: &socket_cutters,
            negative_webbing_exclusions: &[],
            positive_webbing_exclusions: &[],
        },
        Axis::Y,
        &segment_ranges,
        baseline_settings,
    )?;

    let mut mold = generate_sectioned_shell_mold_with_parting_ranges(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
            negative_flange: Some(&lower_flange),
            positive_flange: Some(&upper_flange),
            negative_sockets: &socket_cutters,
            positive_sockets: &socket_cutters,
            negative_webbing_exclusions: &[],
            positive_webbing_exclusions: &[],
        },
        Axis::Y,
        &segment_ranges,
        baseline_settings,
    )?;

    attach_ribs(
        &kernel,
        &part,
        &mut mold.negative,
        &lower_ribs,
        &socket_cutters,
    )?;
    attach_ribs(
        &kernel,
        &part,
        &mut mold.positive,
        &upper_ribs,
        &socket_cutters,
    )?;

    let lower_webbing: Vec<ManifoldSolid> = mold
        .negative
        .iter()
        .zip(&baseline.negative)
        .map(|(webbed, plain)| kernel.difference(webbed, plain))
        .collect::<Result<_, _>>()?;
    let upper_webbing: Vec<ManifoldSolid> = mold
        .positive
        .iter()
        .zip(&baseline.positive)
        .map(|(webbed, plain)| kernel.difference(webbed, plain))
        .collect::<Result<_, _>>()?;
    validate_attached_webbing("lower", &mold.negative, &baseline.negative, &lower_webbing)?;
    validate_attached_webbing("upper", &mold.positive, &baseline.positive, &upper_webbing)?;
    println!(
        "structural webbing: {} lower + {} upper flange-anchored diagonal ribs, {:.2} mm thick ({} x {:.2} mm extrusion), depth {:.1} mm",
        lower_ribs.len(),
        upper_ribs.len(),
        webbing.thickness(),
        webbing.wall_line_count,
        webbing.extrusion_width,
        webbing.depth,
    );

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
    for insert in &inserts {
        kernel.export_stl(&insert.solid, output.join(format!("{}.stl", insert.name)))?;
    }
    for (index, web) in lower_webbing.iter().enumerate() {
        kernel.export_stl(
            web,
            output.join(format!("webbing-lower-{:02}.stl", index + 1)),
        )?;
    }
    for (index, web) in upper_webbing.iter().enumerate() {
        kernel.export_stl(
            web,
            output.join(format!("webbing-upper-{:02}.stl", index + 1)),
        )?;
    }

    let mut assembly =
        Vec::with_capacity(1 + mold.negative.len() + mold.positive.len() + inserts.len());
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
    for insert in &inserts {
        assembly.push(ThreeMfObject {
            name: insert.name.clone(),
            solid: &insert.solid,
        });
    }
    for (index, web) in lower_webbing.iter().enumerate() {
        assembly.push(ThreeMfObject {
            name: format!("webbing-lower-{:02}", index + 1),
            solid: web,
        });
    }
    for (index, web) in upper_webbing.iter().enumerate() {
        assembly.push(ThreeMfObject {
            name: format!("webbing-upper-{:02}", index + 1),
            solid: web,
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

struct RegistrationInsert {
    name: String,
    solid: ManifoldSolid,
}

fn deviation_aware_segment_ranges(
    spec: &WingSpec,
    segment_count: usize,
    outer_min: f64,
    outer_max: f64,
) -> Result<Vec<(f64, f64)>, Box<dyn std::error::Error>> {
    if segment_count == 0 {
        return Err("segment count must be positive".into());
    }
    let first = spec.stations.first().ok_or("wing has no stations")?.span;
    let last = spec.stations.last().ok_or("wing has no stations")?.span;
    let mut candidates: Vec<(f64, f64)> = spec
        .stations
        .windows(3)
        .map(|stations| {
            let incoming = station_axis(stations[0], stations[1]);
            let outgoing = station_axis(stations[1], stations[2]);
            let cosine = dot(incoming, outgoing).clamp(-1.0, 1.0);
            (stations[1].span, cosine.acos())
        })
        .collect();
    candidates.sort_by(|a, b| b.1.total_cmp(&a.1));

    let mut boundaries: Vec<f64> = candidates
        .into_iter()
        .take(segment_count.saturating_sub(1))
        .map(|candidate| candidate.0)
        .collect();
    for index in 1..segment_count {
        if boundaries.len() == segment_count - 1 {
            break;
        }
        let fallback = first + (last - first) * index as f64 / segment_count as f64;
        if !boundaries
            .iter()
            .any(|value| (value - fallback).abs() < 1.0e-6)
        {
            boundaries.push(fallback);
        }
    }
    boundaries.sort_by(f64::total_cmp);
    boundaries.truncate(segment_count.saturating_sub(1));

    let mut edges = Vec::with_capacity(segment_count + 1);
    edges.push(outer_min);
    edges.extend(boundaries);
    edges.push(outer_max);
    Ok(edges.windows(2).map(|edge| (edge[0], edge[1])).collect())
}

fn station_axis(a: WingStation, b: WingStation) -> [f64; 3] {
    let vector = [
        b.x_offset - a.x_offset,
        b.span - a.span,
        b.z_offset - a.z_offset,
    ];
    let length = dot(vector, vector).sqrt();
    [vector[0] / length, vector[1] / length, vector[2] / length]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn registration_inserts(
    spec: &WingSpec,
    segment_ranges: &[(f64, f64)],
    settings: RegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    let mut inserts = Vec::new();

    for (segment, &(start, end)) in segment_ranges.iter().enumerate() {
        for side in [FlangeSide::Leading, FlangeSide::Trailing] {
            let fixture = match side {
                FlangeSide::Leading => settings.leading,
                FlangeSide::Trailing => settings.trailing,
            };
            let spans = fixture_spans(start, end, fixture, settings);
            let side_name = match side {
                FlangeSide::Leading => "leading",
                FlangeSide::Trailing => "trailing",
            };

            println!(
                "segment {} {side_name} flange: {} registration fixtures",
                segment + 1,
                spans.len()
            );

            for (index, center_span) in spans.into_iter().enumerate() {
                let name = format!(
                    "registration-insert-s{:02}-{side_name}-{:02}",
                    segment + 1,
                    index + 1
                );
                let solid = ManifoldSolid(diamond_prism(
                    spec,
                    side,
                    center_span,
                    fixture.chord_half_width,
                    fixture.span_half_width,
                    -fixture.normal_half_depth,
                    fixture.normal_half_depth,
                )?);
                inserts.push(RegistrationInsert { name, solid });
            }
        }
    }

    Ok(inserts)
}

fn fixture_spans(
    flange_start: f64,
    flange_end: f64,
    fixture: FixtureSettings,
    settings: RegistrationSettings,
) -> Vec<f64> {
    let fixture_size = fixture.span_half_width * 2.0;
    let edge_margin = fixture_size * settings.edge_margin_ratio;
    let max_spacing = fixture_size * settings.max_spacing_ratio;
    let usable_start = flange_start + edge_margin;
    let usable_end = flange_end - edge_margin;
    let usable_length = (usable_end - usable_start).max(0.0);
    let spacing_count = if max_spacing > 0.0 {
        (usable_length / max_spacing).ceil() as usize
    } else {
        1
    };
    let count = settings.minimum_per_segment.max(spacing_count + 1);

    (0..count)
        .map(|index| {
            let t = if count == 1 {
                0.5
            } else {
                index as f64 / (count - 1) as f64
            };
            usable_start + usable_length * t
        })
        .collect()
}

fn diagonal_ribs(
    spec: &WingSpec,
    segment_ranges: &[(f64, f64)],
    registration: RegistrationSettings,
    thickness: f64,
    depth: f64,
) -> Result<(Vec<ManifoldSolid>, Vec<ManifoldSolid>), Box<dyn std::error::Error>> {
    let mut lower = Vec::new();
    let mut upper = Vec::new();

    for &(start, end) in segment_ranges {
        let leading = fixture_spans(start, end, registration.leading, registration);
        let trailing = fixture_spans(start, end, registration.trailing, registration);
        let vertex_count = leading.len().min(trailing.len());

        for index in 0..vertex_count.saturating_sub(1) {
            let (a_side, a_span, b_side, b_span) = if index % 2 == 0 {
                (
                    FlangeSide::Leading,
                    leading[index],
                    FlangeSide::Trailing,
                    trailing[index + 1],
                )
            } else {
                (
                    FlangeSide::Trailing,
                    trailing[index],
                    FlangeSide::Leading,
                    leading[index + 1],
                )
            };
            let a = flange_point(spec, a_side, a_span)?;
            let b = flange_point(spec, b_side, b_span)?;
            lower.push(ManifoldSolid(rib_prism(a, b, thickness, -depth)?));
            upper.push(ManifoldSolid(rib_prism(a, b, thickness, depth)?));
        }
    }
    Ok((lower, upper))
}

fn attach_ribs(
    kernel: &ManifoldKernel,
    part: &ManifoldSolid,
    pieces: &mut [ManifoldSolid],
    ribs: &[ManifoldSolid],
    exclusions: &[&ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    for piece in pieces {
        for rib in ribs {
            let mut printable = kernel.difference(rib, part)?;
            for exclusion in exclusions {
                printable = kernel.difference(&printable, exclusion)?;
            }
            *piece = kernel.union_attached(piece, &printable)?;
        }
        // Registration remains the final authoritative operation.
        for exclusion in exclusions {
            *piece = kernel.difference(piece, exclusion)?;
        }
    }
    Ok(())
}

fn validate_attached_webbing(
    half: &str,
    webbed: &[ManifoldSolid],
    baseline: &[ManifoldSolid],
    additions: &[ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    for (index, ((piece, plain), added)) in webbed.iter().zip(baseline).zip(additions).enumerate() {
        let component_count = piece.0.decompose().len();
        let baseline_count = plain.0.decompose().len();
        if component_count > baseline_count {
            return Err(format!(
                "{half} segment {} gained a disconnected webbing component",
                index + 1
            )
            .into());
        }
        if added.0.is_empty() || added.0.volume() <= 1.0e-9 {
            return Err(format!("{half} segment {} has no attached webbing", index + 1).into());
        }
        println!(
            "{half} segment {}: attached webbing volume {:.3} mm^3, {component_count} connected component(s)",
            index + 1,
            added.0.volume(),
        );
    }
    Ok(())
}

fn flange_point(
    spec: &WingSpec,
    side: FlangeSide,
    span: f64,
) -> Result<[f64; 3], Box<dyn std::error::Error>> {
    let station = interpolate_station(spec, span)?;
    Ok(transform_station(
        &station,
        flange_center_x(&station, side),
        0.0,
    ))
}

fn rib_prism(
    start: [f64; 3],
    end: [f64; 3],
    thickness: f64,
    depth: f64,
) -> Result<Manifold, Box<dyn std::error::Error>> {
    let direction = sub(end, start);
    let side = normalize([-direction[1], direction[0], 0.0])?;
    let half = thickness * 0.5;
    let flange_ring = [
        add_scaled(start, side, -half),
        add_scaled(end, side, -half),
        add_scaled(end, side, half),
        add_scaled(start, side, half),
    ];
    let offset_ring = flange_ring.map(|point| [point[0], point[1], point[2] + depth]);
    let (lower, upper) = if depth > 0.0 {
        (flange_ring, offset_ring)
    } else {
        (offset_ring, flange_ring)
    };
    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };
    for point in lower.into_iter().chain(upper) {
        mesh.vert_properties.extend(point);
    }
    connect_ring(&mut mesh, 0, 4);
    mesh.tri_verts.extend([0, 2, 1, 0, 3, 2]);
    mesh.tri_verts.extend([4, 5, 6, 4, 6, 7]);
    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(format!("invalid diagonal rib: {}", solid.status()).into());
    }
    Ok(solid)
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
    mesh.tri_verts.extend([0, 2, 1, 0, 3, 2]);
    mesh.tri_verts.extend([4, 5, 6, 4, 6, 7]);

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
        mesh.tri_verts.extend([lower + i, upper + next, upper + i]);
        mesh.tri_verts
            .extend([lower + i, lower + next, upper + next]);
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
