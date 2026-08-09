use std::fmt::Debug;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds3 {
    pub min: Vec3,
    pub max: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform3 {
    pub matrix: [[f64; 4]; 4],
}

impl Transform3 {
    pub const IDENTITY: Self = Self {
        matrix: [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
    };
}

pub trait SolidKernel {
    type Solid: Clone + Debug;
    type Error: std::error::Error + Send + Sync + 'static;

    fn cuboid(&self, bounds: Bounds3) -> Result<Self::Solid, Self::Error>;
    fn union(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error>;
    fn difference(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error>;
    fn intersection(&self, a: &Self::Solid, b: &Self::Solid) -> Result<Self::Solid, Self::Error>;
    fn transform(&self, solid: &Self::Solid, transform: Transform3) -> Result<Self::Solid, Self::Error>;
}
