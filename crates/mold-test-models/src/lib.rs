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
impl From<std::io::Error> for WingError { fn from(v: std::io::Error) -> Self { Self::Io(v) } }

pub fn preset(name: &str) -> Result<WingSpec, WingError> {
    let airfoil = Naca4::parse("2412")?;
    let linear = |tip_chord: f64, sweep: f64, dihedral: f64, twist: f64| WingSpec {
        airfoil,
        stations: vec![
            WingStation { span: 0.0, chord: 220.0, x_offset: 0.0, z_offset: 0.0, twist_deg: 0.0 },
            WingStation { span: 600.0, chord: tip_chord, x_offset: sweep, z_offset: dihedral, twist_deg: twist },
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
                WingStation { span: 0.0, chord: 240.0, x_offset: 0.0, z_offset: 0.0, twist_deg: 0.0 },
                WingStation { span: 180.0, chord: 205.0, x_offset: 15.0, z_offset: -55.0, twist_deg: -1.0 },
                WingStation { span: 600.0, chord: 125.0, x_offset: 100.0, z_offset: 35.0, twist_deg: -3.0 },
            ],
            profile_points: 48,
            closed_trailing_edge: true,
        }),
        _ => Err(WingError::InvalidSpec("unknown preset")),
    }
}

pub fn generate(spec: &WingSpec) -> Result<Manifold, WingError> {
    if spec.stations.len() < 2 { return Err(WingError::InvalidSpec("at least two stations are required")); }
    if spec.profile_points < 8 { return Err(WingError::InvalidSpec("profile_points must be at least 8")); }
    if spec.stations.windows(2).any(|w| w[1].span <= w[0].span) { return Err(WingError::InvalidSpec("station spans must strictly increase")); }

    let loop_len = spec.profile_points * 2;
    let mut mesh = MeshGL64 { num_prop: 3, ..Default::default() };
    for station in &spec.stations {
        for p in profile(spec.airfoil, spec.profile_points, spec.closed_trailing_edge) {
            let x0 = p.0 * station.chord;
            let z0 = p.1 * station.chord;
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
            mesh.tri_verts.extend([a as u64 + i as u64, b as u64 + i as u64, b as u64 + j as u64]);
            mesh.tri_verts.extend([a as u64 + i as u64, b as u64 + j as u64, a as u64 + j as u64]);
        }
    }
    for i in 1..loop_len - 1 {
        mesh.tri_verts.extend([0, i as u64 + 1, i as u64]);
    }
    let end = (spec.stations.len() - 1) * loop_len;
    for i in 1..loop_len - 1 {
        mesh.tri_verts.extend([end as u64, end as u64 + i as u64, end as u64 + i as u64 + 1]);
    }

    let solid = Manifold::from_mesh_gl64(&mesh);
    if solid.status().to_str() != "No Error" { return Err(WingError::Geometry(solid.status().to_string())); }
    Ok(solid)
}

pub fn write_stl(solid: &Manifold, path: impl AsRef<Path>) -> Result<(), WingError> {
    let mesh = solid.as_original().get_mesh_gl64(-1);
    let stride = mesh.num_prop as usize;
    let vertex = |idx: u64| {
        let o = idx as usize * stride;
        stl_io::Vertex::new([mesh.vert_properties[o] as f32, mesh.vert_properties[o + 1] as f32, mesh.vert_properties[o + 2] as f32])
    };
    let mut tris = Vec::with_capacity(mesh.tri_verts.len() / 3);
    for t in mesh.tri_verts.chunks_exact(3) {
        let vertices = [vertex(t[0]), vertex(t[1]), vertex(t[2])];
        tris.push(stl_io::Triangle { normal: normal(vertices), vertices });
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
        let yt = 5.0 * naca.thickness * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x*x + 0.2843 * x*x*x + te * x*x*x*x);
        let (yc, dy) = camber(naca, x);
        let theta = dy.atan();
        upper.push((x - yt * theta.sin(), yc + yt * theta.cos()));
        lower.push((x + yt * theta.sin(), yc - yt * theta.cos()));
    }
    let mut out = Vec::with_capacity(n * 2);
    out.extend(upper.into_iter().rev());
    out.extend(lower);
    out
}

fn camber(naca: Naca4, x: f64) -> (f64, f64) {
    let m = naca.max_camber;
    let p = naca.camber_position;
    if m == 0.0 || p == 0.0 { return (0.0, 0.0); }
    if x < p {
        (m / (p*p) * (2.0*p*x - x*x), 2.0*m/(p*p)*(p-x))
    } else {
        (m / ((1.0-p)*(1.0-p)) * ((1.0-2.0*p)+2.0*p*x-x*x), 2.0*m/((1.0-p)*(1.0-p))*(p-x))
    }
}

fn normal(v: [stl_io::Vertex; 3]) -> stl_io::Normal {
    let a = [v[1][0]-v[0][0], v[1][1]-v[0][1], v[1][2]-v[0][2]];
    let b = [v[2][0]-v[0][0], v[2][1]-v[0][1], v[2][2]-v[0][2]];
    let n = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
    let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
    stl_io::Normal::new(if l > 0.0 { [n[0]/l,n[1]/l,n[2]/l] } else { [0.0,0.0,0.0] })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn all_presets_generate_closed_manifolds() {
        for name in ["rectangular", "tapered", "swept", "dihedral", "twisted", "gull"] {
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
}
