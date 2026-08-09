use std::num::NonZeroUsize;

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

#[derive(Debug, Clone)]
pub struct SectionedTwoPartMold<S> {
    /// Ordered sections on the negative side of the split axis.
    pub negative: Vec<S>,
    /// Ordered sections on the positive side of the split axis.
    pub positive: Vec<S>,
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

pub fn generate_sectioned_two_part_mold<K>(
    kernel: &K,
    part: &K::Solid,
    settings: MoldSettings,
    split_axis: Axis,
    section_axis: Axis,
    section_count: NonZeroUsize,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let part_bounds = kernel.bounds(part)?;
    let blank_bounds = padded_bounds(part_bounds, settings.margin);
    let split = axis_midpoint(part_bounds, split_axis);
    let (negative_bounds, positive_bounds) = split_bounds(blank_bounds, split_axis, split);
    let section_ranges = section_bounds(blank_bounds, section_axis, section_count);

    let negative = generate_sections(kernel, part, negative_bounds, section_axis, &section_ranges)?;
    let positive = generate_sections(kernel, part, positive_bounds, section_axis, &section_ranges)?;

    Ok(SectionedTwoPartMold { negative, positive })
}

fn generate_sections<K>(
    kernel: &K,
    part: &K::Solid,
    half_bounds: Bounds3,
    section_axis: Axis,
    section_ranges: &[(f64, f64)],
) -> Result<Vec<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let mut sections = Vec::with_capacity(section_ranges.len());

    for &(min, max) in section_ranges {
        let Some(bounds) = clamp_axis_range(half_bounds, section_axis, min, max) else {
            continue;
        };
        let blank = kernel.cuboid(bounds)?;
        sections.push(kernel.difference(&blank, part)?);
    }

    Ok(sections)
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
    let (min, max) = axis_range(bounds, axis);
    (min + max) * 0.5
}

fn axis_range(bounds: Bounds3, axis: Axis) -> (f64, f64) {
    match axis {
        Axis::X => (bounds.min.x, bounds.max.x),
        Axis::Y => (bounds.min.y, bounds.max.y),
        Axis::Z => (bounds.min.z, bounds.max.z),
    }
}

fn split_bounds(bounds: Bounds3, axis: Axis, split: f64) -> (Bounds3, Bounds3) {
    let mut negative = bounds;
    let mut positive = bounds;

    set_axis_max(&mut negative, axis, split);
    set_axis_min(&mut positive, axis, split);

    (negative, positive)
}

fn section_bounds(bounds: Bounds3, axis: Axis, count: NonZeroUsize) -> Vec<(f64, f64)> {
    let (min, max) = axis_range(bounds, axis);
    let width = (max - min) / count.get() as f64;

    (0..count.get())
        .map(|index| {
            let section_min = min + index as f64 * width;
            let section_max = if index + 1 == count.get() {
                max
            } else {
                min + (index + 1) as f64 * width
            };
            (section_min, section_max)
        })
        .collect()
}

fn clamp_axis_range(
    mut bounds: Bounds3,
    axis: Axis,
    range_min: f64,
    range_max: f64,
) -> Option<Bounds3> {
    let (bounds_min, bounds_max) = axis_range(bounds, axis);
    let min = bounds_min.max(range_min);
    let max = bounds_max.min(range_max);

    if max <= min {
        return None;
    }

    set_axis_min(&mut bounds, axis, min);
    set_axis_max(&mut bounds, axis, max);
    Some(bounds)
}

fn set_axis_min(bounds: &mut Bounds3, axis: Axis, value: f64) {
    match axis {
        Axis::X => bounds.min.x = value,
        Axis::Y => bounds.min.y = value,
        Axis::Z => bounds.min.z = value,
    }
}

fn set_axis_max(bounds: &mut Bounds3, axis: Axis, value: f64) {
    match axis {
        Axis::X => bounds.max.x = value,
        Axis::Y => bounds.max.y = value,
        Axis::Z => bounds.max.z = value,
    }
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

    #[test]
    fn section_bounds_cover_axis_without_gaps() {
        let bounds = Bounds3 {
            min: Vec3::new(-10.0, -2.0, -3.0),
            max: Vec3::new(20.0, 4.0, 5.0),
        };
        let count = NonZeroUsize::new(3).unwrap();

        let sections = section_bounds(bounds, Axis::X, count);

        assert_eq!(sections, vec![(-10.0, 0.0), (0.0, 10.0), (10.0, 20.0)]);
    }

    #[test]
    fn clamping_skips_non_overlapping_sections() {
        let bounds = Bounds3 {
            min: Vec3::new(0.0, 0.0, 0.0),
            max: Vec3::new(10.0, 10.0, 10.0),
        };

        assert_eq!(clamp_axis_range(bounds, Axis::X, 10.0, 20.0), None);
        assert_eq!(
            clamp_axis_range(bounds, Axis::X, 5.0, 15.0),
            Some(Bounds3 {
                min: Vec3::new(5.0, 0.0, 0.0),
                max: Vec3::new(10.0, 10.0, 10.0),
            })
        );
    }
}
