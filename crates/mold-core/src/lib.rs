use mold_geometry::{Bounds3, SolidKernel, Vec3};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

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

#[derive(Debug, Clone)]
pub struct TwoPartMold<S> {
    /// Mold part on the negative side of the split axis.
    pub negative: S,
    /// Mold part on the positive side of the split axis.
    pub positive: S,
}

pub fn generate_mold<K>(
    kernel: &K,
    part: &K::Solid,
    settings: MoldSettings,
) -> Result<Mold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let part_bounds = kernel.bounds(part)?;
    let blank_bounds = padded_bounds(part_bounds, settings.margin);
    let blank = kernel.cuboid(blank_bounds)?;
    let body = kernel.difference(&blank, part)?;

    Ok(Mold { body })
}

pub fn generate_two_part_mold<K>(
    kernel: &K,
    part: &K::Solid,
    settings: MoldSettings,
    split_axis: Axis,
) -> Result<TwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let part_bounds = kernel.bounds(part)?;
    let blank_bounds = padded_bounds(part_bounds, settings.margin);
    let split = axis_midpoint(part_bounds, split_axis);
    let (negative_bounds, positive_bounds) = split_bounds(blank_bounds, split_axis, split);

    // Subtract the part from each half independently. Besides avoiding an
    // unnecessary whole-mold boolean, this naturally leaves the cavity open
    // at the split face wherever the source part crosses the split plane.
    let negative_blank = kernel.cuboid(negative_bounds)?;
    let positive_blank = kernel.cuboid(positive_bounds)?;
    let negative = kernel.difference(&negative_blank, part)?;
    let positive = kernel.difference(&positive_blank, part)?;

    Ok(TwoPartMold { negative, positive })
}

fn padded_bounds(part_bounds: Bounds3, margin: Vec3) -> Bounds3 {
    Bounds3 {
        min: Vec3::new(
            part_bounds.min.x - margin.x,
            part_bounds.min.y - margin.y,
            part_bounds.min.z - margin.z,
        ),
        max: Vec3::new(
            part_bounds.max.x + margin.x,
            part_bounds.max.y + margin.y,
            part_bounds.max.z + margin.z,
        ),
    }
}

fn axis_midpoint(bounds: Bounds3, axis: Axis) -> f64 {
    match axis {
        Axis::X => (bounds.min.x + bounds.max.x) * 0.5,
        Axis::Y => (bounds.min.y + bounds.max.y) * 0.5,
        Axis::Z => (bounds.min.z + bounds.max.z) * 0.5,
    }
}

fn split_bounds(bounds: Bounds3, axis: Axis, split: f64) -> (Bounds3, Bounds3) {
    let mut negative = bounds;
    let mut positive = bounds;

    match axis {
        Axis::X => {
            negative.max.x = split;
            positive.min.x = split;
        }
        Axis::Y => {
            negative.max.y = split;
            positive.min.y = split;
        }
        Axis::Z => {
            negative.max.z = split;
            positive.min.z = split;
        }
    }

    (negative, positive)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_bounds_share_the_requested_plane() {
        let bounds = Bounds3 {
            min: Vec3::new(-2.0, -3.0, -4.0),
            max: Vec3::new(8.0, 9.0, 10.0),
        };

        let (negative, positive) = split_bounds(bounds, Axis::Y, 2.5);

        assert_eq!(negative.max.y, 2.5);
        assert_eq!(positive.min.y, 2.5);
        assert_eq!(negative.min, bounds.min);
        assert_eq!(positive.max, bounds.max);
    }
}
