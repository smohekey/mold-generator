//! Manifold-backed geometry implementation.
//!
//! This crate is the first concrete geometry backend for the mold generator.
//! It deliberately owns STL conversion as well as the Manifold adapter so
//! `mold-core` remains independent of both file formats and mesh libraries.

use std::{fmt, fs::File, io::BufWriter, path::Path};

use manifold_rust::{
    linalg::{Mat3x4, Vec3 as ManifoldVec3},
    manifold::Manifold,
    types::{Error as ManifoldError, MeshGL64},
};
use mold_geometry::{Bounds3, SolidKernel, Transform3, Vec3};

#[derive(Clone)]
pub struct ManifoldSolid(pub Manifold);

impl fmt::Debug for ManifoldSolid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ManifoldSolid")
            .field("vertices", &self.0.num_vert())
            .field("triangles", &self.0.num_tri())
            .field("status", &self.0.status())
            .finish()
    }
}

#[derive(Debug)]
pub enum ManifoldKernelError {
    Io(std::io::Error),
    Geometry(ManifoldError),
    InvalidBounds(Bounds3),
    InvalidOffset(f64),
    NonAffineTransform,
}

impl fmt::Display for ManifoldKernelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::Geometry(error) => write!(f, "Manifold geometry error: {error}"),
            Self::InvalidBounds(bounds) => write!(f, "invalid cuboid bounds: {bounds:?}"),
            Self::InvalidOffset(distance) => {
                write!(
                    f,
                    "offset distance must be finite and greater than zero: {distance}"
                )
            }
            Self::NonAffineTransform => write!(f, "transform must be affine"),
        }
    }
}

impl std::error::Error for ManifoldKernelError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            _ => None,
        }
    }
}

impl From<std::io::Error> for ManifoldKernelError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ManifoldKernel;

impl ManifoldKernel {
    pub fn import_stl(&self, path: impl AsRef<Path>) -> Result<ManifoldSolid, ManifoldKernelError> {
        let mut file = File::open(path)?;
        let stl = stl_io::read_stl(&mut file)?;

        let mut mesh = MeshGL64 {
            num_prop: 3,
            ..Default::default()
        };

        mesh.vert_properties.reserve(stl.vertices.len() * 3);
        for vertex in &stl.vertices {
            mesh.vert_properties
                .extend([vertex[0] as f64, vertex[1] as f64, vertex[2] as f64]);
        }

        mesh.tri_verts.reserve(stl.faces.len() * 3);
        for face in &stl.faces {
            mesh.tri_verts
                .extend(face.vertices.map(|index| index as u64));
        }

        let solid = Manifold::from_mesh_gl64(&mesh);
        self.checked(solid)
    }

    pub fn export_stl(
        &self,
        solid: &ManifoldSolid,
        path: impl AsRef<Path>,
    ) -> Result<(), ManifoldKernelError> {
        self.ensure_ok(&solid.0)?;

        let mesh = solid.0.as_original().get_mesh_gl64(-1);
        let stride = mesh.num_prop as usize;

        let vertex_at = |index: u64| {
            let offset = index as usize * stride;
            stl_io::Vertex::new([
                mesh.vert_properties[offset] as f32,
                mesh.vert_properties[offset + 1] as f32,
                mesh.vert_properties[offset + 2] as f32,
            ])
        };

        let mut triangles = Vec::with_capacity(mesh.tri_verts.len() / 3);
        for indices in mesh.tri_verts.chunks_exact(3) {
            let vertices = [
                vertex_at(indices[0]),
                vertex_at(indices[1]),
                vertex_at(indices[2]),
            ];
            let normal = triangle_normal(vertices[0], vertices[1], vertices[2]);
            triangles.push(stl_io::Triangle { normal, vertices });
        }

        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        stl_io::write_stl(&mut writer, triangles.iter())?;
        Ok(())
    }

    fn checked(&self, manifold: Manifold) -> Result<ManifoldSolid, ManifoldKernelError> {
        self.ensure_ok(&manifold)?;
        Ok(ManifoldSolid(manifold))
    }

    fn ensure_ok(&self, manifold: &Manifold) -> Result<(), ManifoldKernelError> {
        match manifold.status() {
            ManifoldError::NoError => Ok(()),
            error => Err(ManifoldKernelError::Geometry(error)),
        }
    }
}

impl SolidKernel for ManifoldKernel {
    type Solid = ManifoldSolid;
    type Error = ManifoldKernelError;

    fn bounds(&self, solid: &Self::Solid) -> Result<Bounds3, Self::Error> {
        self.ensure_ok(&solid.0)?;
        let bounds = solid.0.bounding_box();
        Ok(Bounds3 {
            min: Vec3::new(bounds.min.x, bounds.min.y, bounds.min.z),
            max: Vec3::new(bounds.max.x, bounds.max.y, bounds.max.z),
        })
    }

    fn cuboid(&self, bounds: Bounds3) -> Result<Self::Solid, Self::Error> {
        let size = ManifoldVec3::new(
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
        );

        if !size.x.is_finite()
            || !size.y.is_finite()
            || !size.z.is_finite()
            || size.x <= 0.0
            || size.y <= 0.0
            || size.z <= 0.0
        {
            return Err(ManifoldKernelError::InvalidBounds(bounds));
        }

        let solid = Manifold::cube(size, false).translate(ManifoldVec3::new(
            bounds.min.x,
            bounds.min.y,
            bounds.min.z,
        ));
        self.checked(solid)
    }

    fn union(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error> {
        self.checked(a.0.union(&b.0))
    }

    fn difference(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error> {
        self.checked(a.0.difference(&b.0))
    }

    fn intersection(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error> {
        self.checked(a.0.intersection(&b.0))
    }

    fn offset(&self, solid: &Self::Solid, distance: f64) -> Result<Self::Solid, Self::Error> {
        if !distance.is_finite() || distance <= 0.0 {
            return Err(ManifoldKernelError::InvalidOffset(distance));
        }

        // The Minkowski sum with a sphere is the geometric dilation of the
        // source solid. Sixteen circular segments keeps the initial mesh
        // backend reasonably light while producing a smooth enough mold skin;
        // this can become an explicit quality setting later.
        let kernel = Manifold::sphere(distance, 16);
        self.checked(solid.0.minkowski_sum(&kernel))
    }

    fn transform(
        &self,
        solid: &Self::Solid,
        transform: Transform3,
    ) -> Result<Self::Solid, Self::Error> {
        let m = transform.matrix;
        if m[3] != [0.0, 0.0, 0.0, 1.0] {
            return Err(ManifoldKernelError::NonAffineTransform);
        }

        let manifold_transform = Mat3x4::from_cols(
            ManifoldVec3::new(m[0][0], m[1][0], m[2][0]),
            ManifoldVec3::new(m[0][1], m[1][1], m[2][1]),
            ManifoldVec3::new(m[0][2], m[1][2], m[2][2]),
            ManifoldVec3::new(m[0][3], m[1][3], m[2][3]),
        );
        self.checked(solid.0.transform(&manifold_transform))
    }
}

fn triangle_normal(a: stl_io::Vertex, b: stl_io::Vertex, c: stl_io::Vertex) -> stl_io::Normal {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let normal = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();

    if length > 0.0 {
        stl_io::Normal::new([normal[0] / length, normal[1] / length, normal[2] / length])
    } else {
        stl_io::Normal::new([0.0, 0.0, 0.0])
    }
}
