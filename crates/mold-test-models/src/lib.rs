use std::{fmt, fs::File, io::BufWriter, path::Path};

use manifold_rust::{manifold::Manifold, types::MeshGL64};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Naca4 {
    pub max_camber: f64,
    pub camber_position: f64,
    pub thickness: f64,
}

impl Naca4 {
    pub fn parse(code: &str) -> Result<Self, WingError> {
        if code.len() != 4 || !code.bytes().all(|b| b.is_ascii_digit()) {
            return Err(WingError::InvalidAirfoil(code.to_owned()));
        }
        let digits: Vec<u32> = code.chars().map(|c| c.to_digit(10).unwrap()).collect();
        Ok(Self {
            max_camber: digits[0] as f64 / 100.0,
            camber_position: digits[1] as f64 / 10.0,
            thickness: (digits[2] * 10 + digits[3]) as f64 / 100.0,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WingStation {
    pub span: f64,
    pub chord: f64,
    pub x_offset: f64,
    pub z_offset: f64,
    pub twist_deg: f64,
}

#[derive(Debug, Clone)]
pub struct WingSpec {
    pub airfoil: Naca4,
    pub stations: Vec<WingStation>,
    pub profile_points: usize,
    pub closed_trailing_edge: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WingEdge {
    Leading,
    Trailing,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WingSurface {
    Lower,
    Upper,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RibPathSpec {
    pub start_edge: WingEdge,
    pub start_span: f64,
    pub end_edge: WingEdge,
    pub end_span: f64,
    pub flange_margin: f64,
    pub rib_thickness: f64,
    pub samples: usize,
    pub surface: WingSurface,
}

pub fn deviation_aware_segment_ranges(
    spec: &WingSpec,
    segment_count: usize,
    outer_min: f64,
    outer_max: f64,
) -> Result<Vec<(f64, f64)>, WingError> {
    if segment_count == 0 {
        return Err(WingError::InvalidSpec("segment count must be positive"));
    }
    let first = spec
        .stations
        .first()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let last = spec
        .stations
        .last()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let mut candidates: Vec<(f64, f64)> = spec
        .stations
        .windows(3)
        .map(|stations| {
            let incoming = station_axis(stations[0], stations[1]);
            let outgoing = station_axis(stations[1], stations[2]);
            (
                stations[1].span,
                dot(incoming, outgoing).clamp(-1.0, 1.0).acos(),
            )
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

pub fn segment_normal(
    spec: &WingSpec,
    start_span: f64,
    end_span: f64,
) -> Result<[f64; 3], WingError> {
    let model_start = spec
        .stations
        .first()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let model_end = spec
        .stations
        .last()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let start = interpolate_station(spec, start_span.max(model_start))?;
    let end = interpolate_station(spec, end_span.min(model_end))?;
    let center = interpolate_station(spec, (start.span + end.span) * 0.5)?;
    let chord_tangent = subtract(
        transform_station(&center, center.chord, 0.0),
        transform_station(&center, 0.0, 0.0),
    );
    let span_tangent = subtract(
        transform_station(&end, end.chord * 0.5, 0.0),
        transform_station(&start, start.chord * 0.5, 0.0),
    );
    let mut normal = normalize_array(cross_array(chord_tangent, span_tangent))
        .ok_or(WingError::InvalidSpec("cannot determine segment normal"))?;
    if normal[2] < 0.0 {
        normal = [-normal[0], -normal[1], -normal[2]];
    }
    Ok(normal)
}

pub fn sample_rib_surface_path(
    spec: &WingSpec,
    path: RibPathSpec,
) -> Result<Vec<[f64; 3]>, WingError> {
    if path.samples == 0 {
        return Err(WingError::InvalidSpec("rib samples must be positive"));
    }
    (0..=path.samples)
        .map(|index| {
            let t = index as f64 / path.samples as f64;
            let station = interpolate_station(spec, lerp(path.start_span, path.end_span, t))?;
            let start_x = rib_endpoint_x(
                &station,
                path.start_edge,
                path.flange_margin,
                path.rib_thickness,
            );
            let end_x = rib_endpoint_x(
                &station,
                path.end_edge,
                path.flange_margin,
                path.rib_thickness,
            );
            surface_point(spec, &station, lerp(start_x, end_x, t), path.surface)
        })
        .collect()
}

pub fn interpolate_station(spec: &WingSpec, span: f64) -> Result<WingStation, WingError> {
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
    Err(WingError::Geometry(format!(
        "span {span} is outside the wing"
    )))
}

pub fn transform_station(station: &WingStation, x: f64, z: f64) -> [f64; 3] {
    let pivot = station.chord * 0.25;
    let angle = station.twist_deg.to_radians();
    let dx = x - pivot;
    [
        pivot + dx * angle.cos() + z * angle.sin() + station.x_offset,
        station.span,
        -dx * angle.sin() + z * angle.cos() + station.z_offset,
    ]
}

pub fn chord_region(
    spec: &WingSpec,
    z_min: f64,
    z_max: f64,
    chord_margin: f64,
) -> Result<Manifold, WingError> {
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
    close_quad_loft(&mut mesh, spec.stations.len());
    checked_mesh(mesh, "chord region")
}

pub fn registration_diamond(
    spec: &WingSpec,
    edge: WingEdge,
    center_span: f64,
    chord_half_width: f64,
    span_half_width: f64,
    normal_half_depth: f64,
) -> Result<Manifold, WingError> {
    let center = interpolate_station(spec, center_span)?;
    let inboard = interpolate_station(spec, center_span - span_half_width)?;
    let outboard = interpolate_station(spec, center_span + span_half_width)?;
    let footprint = [
        transform_station(
            &center,
            flange_center_x(&center, edge) - chord_half_width,
            0.0,
        ),
        transform_station(&inboard, flange_center_x(&inboard, edge), 0.0),
        transform_station(
            &center,
            flange_center_x(&center, edge) + chord_half_width,
            0.0,
        ),
        transform_station(&outboard, flange_center_x(&outboard, edge), 0.0),
    ];
    let normal = flange_normal(spec, edge, center_span)?;
    let base = footprint.map(|point| add_scaled(point, normal, -normal_half_depth));
    let top = footprint.map(|point| add_scaled(point, normal, normal_half_depth));
    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };
    for point in base.into_iter().chain(top) {
        mesh.vert_properties.extend(point);
    }
    connect_quad_rings(&mut mesh, 0, 4);
    mesh.tri_verts.extend([0, 2, 1, 0, 3, 2]);
    mesh.tri_verts.extend([4, 5, 6, 4, 6, 7]);
    checked_mesh(mesh, "registration diamond")
}

fn surface_point(
    spec: &WingSpec,
    station: &WingStation,
    x: f64,
    surface: WingSurface,
) -> Result<[f64; 3], WingError> {
    if x <= 0.0 || x >= station.chord {
        return Ok(transform_station(station, x, 0.0));
    }
    let normalized_x = x / station.chord;
    let trailing = if spec.closed_trailing_edge {
        -0.1036
    } else {
        -0.1015
    };
    let thickness = 5.0
        * spec.airfoil.thickness
        * (0.2969 * normalized_x.sqrt() - 0.1260 * normalized_x - 0.3516 * normalized_x.powi(2)
            + 0.2843 * normalized_x.powi(3)
            + trailing * normalized_x.powi(4));
    let (camber, _) = camber(spec.airfoil, normalized_x);
    let signed_thickness = match surface {
        WingSurface::Lower => -thickness,
        WingSurface::Upper => thickness,
    };
    Ok(transform_station(
        station,
        x,
        (camber + signed_thickness) * station.chord,
    ))
}

fn rib_endpoint_x(
    station: &WingStation,
    edge: WingEdge,
    flange_margin: f64,
    thickness: f64,
) -> f64 {
    let half = thickness * 0.5;
    match edge {
        WingEdge::Leading => -flange_margin + half,
        WingEdge::Trailing => station.chord + flange_margin - half,
    }
}

fn station_axis(a: WingStation, b: WingStation) -> [f64; 3] {
    normalize_array([
        b.x_offset - a.x_offset,
        b.span - a.span,
        b.z_offset - a.z_offset,
    ])
    .expect("stations have strictly increasing spans")
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn subtract(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross_array(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize_array(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot(vector, vector).sqrt();
    (length > f64::EPSILON).then(|| [vector[0] / length, vector[1] / length, vector[2] / length])
}

fn flange_center_x(station: &WingStation, edge: WingEdge) -> f64 {
    match edge {
        WingEdge::Leading => -6.0,
        WingEdge::Trailing => station.chord + 6.0,
    }
}

fn flange_normal(spec: &WingSpec, edge: WingEdge, span: f64) -> Result<[f64; 3], WingError> {
    let center = interpolate_station(spec, span)?;
    let inboard = interpolate_station(spec, span - 1.0)?;
    let outboard = interpolate_station(spec, span + 1.0)?;
    let center_x = flange_center_x(&center, edge);
    let chord_tangent = subtract(
        transform_station(&center, center_x + 1.0, 0.0),
        transform_station(&center, center_x - 1.0, 0.0),
    );
    let span_tangent = subtract(
        transform_station(&outboard, flange_center_x(&outboard, edge), 0.0),
        transform_station(&inboard, flange_center_x(&inboard, edge), 0.0),
    );
    let mut normal = normalize_array(cross_array(chord_tangent, span_tangent))
        .ok_or(WingError::InvalidSpec("cannot determine flange normal"))?;
    if normal[2] < 0.0 {
        normal = [-normal[0], -normal[1], -normal[2]];
    }
    Ok(normal)
}

fn add_scaled(point: [f64; 3], direction: [f64; 3], scale: f64) -> [f64; 3] {
    [
        point[0] + direction[0] * scale,
        point[1] + direction[1] * scale,
        point[2] + direction[2] * scale,
    ]
}

fn connect_quad_rings(mesh: &mut MeshGL64, lower: u64, upper: u64) {
    for index in 0..4_u64 {
        let next = (index + 1) % 4;
        mesh.tri_verts
            .extend([lower + index, upper + next, upper + index]);
        mesh.tri_verts
            .extend([lower + index, lower + next, upper + next]);
    }
}

fn close_quad_loft(mesh: &mut MeshGL64, stations: usize) {
    for station in 0..stations - 1 {
        let a = (station * 4) as u64;
        let b = ((station + 1) * 4) as u64;
        for index in 0..4_u64 {
            let next = (index + 1) % 4;
            mesh.tri_verts.extend([a + index, b + index, b + next]);
            mesh.tri_verts.extend([a + index, b + next, a + next]);
        }
    }
    mesh.tri_verts.extend([0, 1, 2, 0, 2, 3]);
    let end = ((stations - 1) * 4) as u64;
    mesh.tri_verts
        .extend([end, end + 2, end + 1, end, end + 3, end + 2]);
}

fn checked_mesh(mesh: MeshGL64, label: &str) -> Result<Manifold, WingError> {
    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(WingError::Geometry(format!(
            "invalid {label}: {}",
            solid.status()
        )));
    }
    Ok(solid)
}

#[derive(Debug)]
pub enum WingError {
    InvalidAirfoil(String),
    InvalidSpec(&'static str),
    Io(std::io::Error),
    Geometry(String),
}

impl fmt::Display for WingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAirfoil(code) => write!(f, "invalid NACA 4-digit airfoil: {code}"),
            Self::InvalidSpec(msg) => write!(f, "invalid wing specification: {msg}"),
            Self::Io(err) => err.fmt(f),
            Self::Geometry(msg) => write!(f, "generated geometry is invalid: {msg}"),
        }
    }
}

impl std::error::Error for WingError {}

impl From<std::io::Error> for WingError {
    fn from(v: std::io::Error) -> Self {
        Self::Io(v)
    }
}

pub fn preset(name: &str) -> Result<WingSpec, WingError> {
    let airfoil = Naca4::parse("2412")?;
    let linear = |tip_chord: f64, sweep: f64, dihedral: f64, twist: f64| WingSpec {
        airfoil,
        stations: vec![
            WingStation {
                span: 0.0,
                chord: 220.0,
                x_offset: 0.0,
                z_offset: 0.0,
                twist_deg: 0.0,
            },
            WingStation {
                span: 600.0,
                chord: tip_chord,
                x_offset: sweep,
                z_offset: dihedral,
                twist_deg: twist,
            },
        ],
        profile_points: 48,
        closed_trailing_edge: true,
    };
    match name {
        "rectangular" => Ok(linear(220.0, 0.0, 0.0, 0.0)),
        "tapered" => Ok(linear(120.0, 0.0, 0.0, 0.0)),
        "swept" => Ok(linear(140.0, 120.0, 0.0, 0.0)),
        "dihedral" => Ok(linear(160.0, 0.0, 70.0, 0.0)),
        "twisted" => Ok(linear(130.0, 70.0, 45.0, -4.0)),
        "gull" => Ok(WingSpec {
            airfoil,
            stations: vec![
                WingStation {
                    span: 0.0,
                    chord: 240.0,
                    x_offset: 0.0,
                    z_offset: 0.0,
                    twist_deg: 0.0,
                },
                WingStation {
                    span: 180.0,
                    chord: 205.0,
                    x_offset: 15.0,
                    z_offset: -55.0,
                    twist_deg: -1.0,
                },
                WingStation {
                    span: 600.0,
                    chord: 125.0,
                    x_offset: 100.0,
                    z_offset: 35.0,
                    twist_deg: -3.0,
                },
            ],
            profile_points: 48,
            closed_trailing_edge: true,
        }),
        _ => Err(WingError::InvalidSpec("unknown preset")),
    }
}

pub fn generate(spec: &WingSpec) -> Result<Manifold, WingError> {
    if spec.stations.len() < 2 {
        return Err(WingError::InvalidSpec("at least two stations are required"));
    }
    if spec.profile_points < 8 {
        return Err(WingError::InvalidSpec("profile_points must be at least 8"));
    }
    if spec.stations.windows(2).any(|w| w[1].span <= w[0].span) {
        return Err(WingError::InvalidSpec(
            "station spans must strictly increase",
        ));
    }

    let section_profile = profile(spec.airfoil, spec.profile_points, spec.closed_trailing_edge);
    let loop_len = section_profile.len();
    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };
    for station in &spec.stations {
        for &(profile_x, profile_z) in &section_profile {
            let x0 = profile_x * station.chord;
            let z0 = profile_z * station.chord;
            let pivot = 0.25 * station.chord;
            let a = station.twist_deg.to_radians();
            let dx = x0 - pivot;
            let x = pivot + dx * a.cos() + z0 * a.sin() + station.x_offset;
            let z = -dx * a.sin() + z0 * a.cos() + station.z_offset;
            mesh.vert_properties.extend([x, station.span, z]);
        }
    }

    for s in 0..spec.stations.len() - 1 {
        let a = s * loop_len;
        let b = (s + 1) * loop_len;
        for i in 0..loop_len {
            let j = (i + 1) % loop_len;
            mesh.tri_verts.extend([
                a as u64 + i as u64,
                b as u64 + i as u64,
                b as u64 + j as u64,
            ]);
            mesh.tri_verts.extend([
                a as u64 + i as u64,
                b as u64 + j as u64,
                a as u64 + j as u64,
            ]);
        }
    }

    // The section profile is wound TE -> upper -> LE -> lower -> TE. With
    // span increasing along +Y, the root cap therefore needs -Y normals and
    // the tip cap +Y normals to remain consistent with the side faces.
    for i in 1..loop_len - 1 {
        mesh.tri_verts.extend([0, i as u64, i as u64 + 1]);
    }
    let end = (spec.stations.len() - 1) * loop_len;
    for i in 1..loop_len - 1 {
        mesh.tri_verts
            .extend([end as u64, end as u64 + i as u64 + 1, end as u64 + i as u64]);
    }

    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" {
        return Err(WingError::Geometry(solid.status().to_string()));
    }
    Ok(solid)
}

pub fn write_stl(solid: &Manifold, path: impl AsRef<Path>) -> Result<(), WingError> {
    let mesh = solid.as_original().get_mesh_gl64(-1);
    let stride = mesh.num_prop as usize;
    let vertex = |idx: u64| {
        let o = idx as usize * stride;
        stl_io::Vertex::new([
            mesh.vert_properties[o] as f32,
            mesh.vert_properties[o + 1] as f32,
            mesh.vert_properties[o + 2] as f32,
        ])
    };
    let mut tris = Vec::with_capacity(mesh.tri_verts.len() / 3);
    for t in mesh.tri_verts.chunks_exact(3) {
        let vertices = [vertex(t[0]), vertex(t[1]), vertex(t[2])];
        tris.push(stl_io::Triangle {
            normal: normal(vertices),
            vertices,
        });
    }
    let mut out = BufWriter::new(File::create(path)?);
    stl_io::write_stl(&mut out, tris.iter())?;
    Ok(())
}

fn profile(naca: Naca4, n: usize, closed_te: bool) -> Vec<(f64, f64)> {
    let mut upper = Vec::with_capacity(n);
    let mut lower = Vec::with_capacity(n);
    for i in 0..n {
        let beta = std::f64::consts::PI * i as f64 / (n - 1) as f64;
        let x = 0.5 * (1.0 - beta.cos());
        let te = if closed_te { -0.1036 } else { -0.1015 };
        let yt = 5.0
            * naca.thickness
            * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x
                + 0.2843 * x * x * x
                + te * x * x * x * x);
        let (yc, dy) = camber(naca, x);
        let theta = dy.atan();
        upper.push((x - yt * theta.sin(), yc + yt * theta.cos()));
        lower.push((x + yt * theta.sin(), yc - yt * theta.cos()));
    }

    // Walk TE -> LE on the upper surface, then LE -> TE on the lower surface.
    // The leading and trailing edge vertices are shared, not duplicated, so
    // each loft station is one topological loop rather than coincident edges.
    let mut out = Vec::with_capacity(2 * n - 2);
    out.extend(upper.into_iter().rev());
    out.extend(lower.into_iter().skip(1).take(n - 2));
    out
}

fn camber(naca: Naca4, x: f64) -> (f64, f64) {
    let m = naca.max_camber;
    let p = naca.camber_position;
    if m == 0.0 || p == 0.0 {
        return (0.0, 0.0);
    }
    if x < p {
        (
            m / (p * p) * (2.0 * p * x - x * x),
            2.0 * m / (p * p) * (p - x),
        )
    } else {
        (
            m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x),
            2.0 * m / ((1.0 - p) * (1.0 - p)) * (p - x),
        )
    }
}

fn normal(v: [stl_io::Vertex; 3]) -> stl_io::Normal {
    let a = [v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]];
    let b = [v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]];
    let n = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    stl_io::Normal::new(if l > 0.0 {
        [n[0] / l, n[1] / l, n[2] / l]
    } else {
        [0.0, 0.0, 0.0]
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_presets_generate_closed_manifolds() {
        for name in [
            "rectangular",
            "tapered",
            "swept",
            "dihedral",
            "twisted",
            "gull",
        ] {
            let solid = generate(&preset(name).unwrap()).unwrap();
            assert!(!solid.is_empty(), "{name}");
            assert!(solid.volume() > 0.0, "{name}");
        }
    }

    #[test]
    fn naca_0012_is_symmetric() {
        let n = Naca4::parse("0012").unwrap();
        assert_eq!(n.max_camber, 0.0);
        assert_eq!(n.thickness, 0.12);
    }

    #[test]
    fn gull_segments_break_at_the_largest_axial_deviation() {
        let spec = preset("gull").unwrap();
        let ranges = deviation_aware_segment_ranges(&spec, 2, -3.0, 363.0).unwrap();

        assert_eq!(ranges, vec![(-3.0, 180.0), (180.0, 363.0)]);
    }

    #[test]
    fn segment_normal_tracks_the_wing_and_points_upward() {
        let spec = preset("gull").unwrap();
        let normal = segment_normal(&spec, 0.0, 180.0).unwrap();
        let length = normal
            .iter()
            .map(|component| component.powi(2))
            .sum::<f64>()
            .sqrt();

        assert!((length - 1.0).abs() < 1.0e-12);
        assert!(normal[2] > 0.0);
        assert!(normal[1].abs() > 0.05);
    }

    #[test]
    fn sampled_rib_paths_follow_each_airfoil_surface_to_the_flange_edges() {
        let spec = preset("gull").unwrap();
        let lower_spec = RibPathSpec {
            start_edge: WingEdge::Leading,
            start_span: 40.0,
            end_edge: WingEdge::Trailing,
            end_span: 140.0,
            flange_margin: 12.0,
            rib_thickness: 1.35,
            samples: 24,
            surface: WingSurface::Lower,
        };
        let lower = sample_rib_surface_path(&spec, lower_spec).unwrap();
        let upper = sample_rib_surface_path(
            &spec,
            RibPathSpec {
                surface: WingSurface::Upper,
                ..lower_spec
            },
        )
        .unwrap();

        assert_eq!(lower.len(), 25);
        assert_eq!(upper.len(), 25);
        assert!(upper[12][2] > lower[12][2]);
        assert!(lower[0][0] < 0.0);
        assert!(lower[24][0] > spec.stations[1].chord);
    }

    #[test]
    fn gull_chord_regions_are_closed_manifolds() {
        let spec = preset("gull").unwrap();

        for region in [
            chord_region(&spec, -500.0, 0.0, 80.0).unwrap(),
            chord_region(&spec, 0.0, 500.0, 80.0).unwrap(),
            chord_region(&spec, -3.0, 0.0, 12.0).unwrap(),
            chord_region(&spec, 0.0, 3.0, 12.0).unwrap(),
        ] {
            assert!(!region.is_empty());
            assert!(region.volume() > 0.0);
        }
    }
}
