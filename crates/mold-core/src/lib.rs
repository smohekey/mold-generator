use mold_geometry::{Bounds3, SolidKernel, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoldSettings {
    /// Extra material around the part on each side, in model units.
    pub margin: Vec3,
}

impl Default for MoldSettings {
    fn default() -> Self {
        Self {
            margin: Vec3::new(10.0, 10.0, 10.0),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Mold<S> {
    pub body: S,
}

pub fn generate_mold<K>(
    kernel: &K,
    part: &K::Solid,
    part_bounds: Bounds3,
    settings: MoldSettings,
) -> Result<Mold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let blank_bounds = Bounds3 {
        min: Vec3::new(
            part_bounds.min.x - settings.margin.x,
            part_bounds.min.y - settings.margin.y,
            part_bounds.min.z - settings.margin.z,
        ),
        max: Vec3::new(
            part_bounds.max.x + settings.margin.x,
            part_bounds.max.y + settings.margin.y,
            part_bounds.max.z + settings.margin.z,
        ),
    };

    let blank = kernel.cuboid(blank_bounds)?;
    let body = kernel.difference(&blank, part)?;

    Ok(Mold { body })
}
