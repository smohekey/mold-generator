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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionRegistration {
    /// Distance a male key extends into the neighboring section.
    pub depth: f64,
    /// Key size along the non-section, non-split axis.
    pub width: f64,
    /// Key size along the split axis.
    pub height: f64,
    /// Gap added around the matching female socket.
    pub clearance: f64,
    /// Distance between the key and the exterior mold face on the split axis.
    pub edge_inset: f64,
}

impl Default for SectionRegistration {
    fn default() -> Self {
        Self {
            depth: 4.0,
            width: 10.0,
            height: 4.0,
            clearance: 0.2,
            edge_inset: 2.0,
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
    generate_sectioned_two_part_mold_impl(
        kernel,
        part,
        settings,
        split_axis,
        section_axis,
        section_count,
        None,
    )
}

pub fn generate_registered_sectioned_two_part_mold<K>(
    kernel: &K,
    part: &K::Solid,
    settings: MoldSettings,
    split_axis: Axis,
    section_axis: Axis,
    section_count: NonZeroUsize,
    registration: SectionRegistration,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    generate_sectioned_two_part_mold_impl(
        kernel,
        part,
        settings,
        split_axis,
        section_axis,
        section_count,
        Some(registration),
    )
}

fn generate_sectioned_two_part_mold_impl<K>(
    kernel: &K,
    part: &K::Solid,
    settings: MoldSettings,
    split_axis: Axis,
    section_axis: Axis,
    section_count: NonZeroUsize,
    registration: Option<SectionRegistration>,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let part_bounds = kernel.bounds(part)?;
    let blank_bounds = padded_bounds(part_bounds, settings.margin);
    let split = axis_midpoint(part_bounds, split_axis);
    let (negative_bounds, positive_bounds) = split_bounds(blank_bounds, split_axis, split);
    let section_ranges = section_bounds(blank_bounds, section_axis, section_count);

    let negative = generate_sections(
        kernel,
        part,
        negative_bounds,
        split_axis,
        section_axis,
        &section_ranges,
        registration.map(|settings| (settings, HalfSide::Negative)),
    )?;
    let positive = generate_sections(
        kernel,
        part,
        positive_bounds,
        split_axis,
        section_axis,
        &section_ranges,
        registration.map(|settings| (settings, HalfSide::Positive)),
    )?;

    Ok(SectionedTwoPartMold { negative, positive })
}

#[derive(Debug, Clone, Copy)]
enum HalfSide {
    Negative,
    Positive,
}

fn generate_sections<K>(
    kernel: &K,
    part: &K::Solid,
    half_bounds: Bounds3,
    split_axis: Axis,
    section_axis: Axis,
    section_ranges: &[(f64, f64)],
    registration: Option<(SectionRegistration, HalfSide)>,
) -> Result<Vec<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let mut sections = Vec::with_capacity(section_ranges.len());

    for (index, &(min, max)) in section_ranges.iter().enumerate() {
        let Some(bounds) = clamp_axis_range(half_bounds, section_axis, min, max) else {
            continue;
        };
        let blank = kernel.cuboid(bounds)?;
        let mut section = kernel.difference(&blank, part)?;

        if let Some((registration, side)) = registration
            && split_axis != section_axis
        {
            if index + 1 < section_ranges.len() {
                for key_bounds in registration_key_bounds(
                    half_bounds,
                    split_axis,
                    section_axis,
                    max,
                    registration,
                    side,
                    RegistrationKind::Male,
                ) {
                    let key = kernel.cuboid(key_bounds)?;
                    section = kernel.union(&section, &key)?;
                }
            }

            if index > 0 {
                for socket_bounds in registration_key_bounds(
                    half_bounds,
                    split_axis,
                    section_axis,
                    min,
                    registration,
                    side,
                    RegistrationKind::Female,
                ) {
                    let socket = kernel.cuboid(socket_bounds)?;
                    section = kernel.difference(&section, &socket)?;
                }
            }
        }

        sections.push(section);
    }

    Ok(sections)
}

#[derive(Debug, Clone, Copy)]
enum RegistrationKind {
    Male,
    Female,
}

fn registration_key_bounds(
    half_bounds: Bounds3,
    split_axis: Axis,
    section_axis: Axis,
    interface: f64,
    settings: SectionRegistration,
    side: HalfSide,
    kind: RegistrationKind,
) -> Vec<Bounds3> {
    let transverse_axis = remaining_axis(split_axis, section_axis);
    let (transverse_min, transverse_max) = axis_range(half_bounds, transverse_axis);
    let transverse_span = transverse_max - transverse_min;
    let clearance = match kind {
        RegistrationKind::Male => 0.0,
        RegistrationKind::Female => settings.clearance,
    };

    [0.3, 0.7]
        .into_iter()
        .map(|fraction| {
            let center = transverse_min + transverse_span * fraction;
            let half_width = settings.width * 0.5 + clearance;
            let mut bounds = half_bounds;
            set_axis_min(&mut bounds, transverse_axis, center - half_width);
            set_axis_max(&mut bounds, transverse_axis, center + half_width);

            let (split_min, split_max) = axis_range(half_bounds, split_axis);
            match side {
                HalfSide::Negative => {
                    set_axis_min(
                        &mut bounds,
                        split_axis,
                        split_min + settings.edge_inset - clearance,
                    );
                    set_axis_max(
                        &mut bounds,
                        split_axis,
                        split_min + settings.edge_inset + settings.height + clearance,
                    );
                }
                HalfSide::Positive => {
                    set_axis_min(
                        &mut bounds,
                        split_axis,
                        split_max - settings.edge_inset - settings.height - clearance,
                    );
                    set_axis_max(
                        &mut bounds,
                        split_axis,
                        split_max - settings.edge_inset + clearance,
                    );
                }
            }

            match kind {
                RegistrationKind::Male => {
                    let embed = settings.depth * 0.25;
                    set_axis_min(&mut bounds, section_axis, interface - embed);
                    set_axis_max(&mut bounds, section_axis, interface + settings.depth);
                }
                RegistrationKind::Female => {
                    set_axis_min(&mut bounds, section_axis, interface);
                    set_axis_max(
                        &mut bounds,
                        section_axis,
                        interface + settings.depth + settings.clearance,
                    );
                }
            }

            bounds
        })
        .collect()
}

fn remaining_axis(a: Axis, b: Axis) -> Axis {
    match (a, b) {
        (Axis::X, Axis::Y) | (Axis::Y, Axis::X) => Axis::Z,
        (Axis::X, Axis::Z) | (Axis::Z, Axis::X) => Axis::Y,
        (Axis::Y, Axis::Z) | (Axis::Z, Axis::Y) => Axis::X,
        _ => a,
    }
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

    #[test]
    fn registration_keys_are_placed_on_the_exterior_side_of_each_half() {
        let bounds = Bounds3 {
            min: Vec3::new(0.0, 0.0, -20.0),
            max: Vec3::new(100.0, 200.0, 0.0),
        };
        let settings = SectionRegistration::default();
        let keys = registration_key_bounds(
            bounds,
            Axis::Z,
            Axis::Y,
            50.0,
            settings,
            HalfSide::Negative,
            RegistrationKind::Male,
        );

        assert_eq!(keys.len(), 2);
        assert_eq!(keys[0].min.z, -18.0);
        assert_eq!(keys[0].max.z, -14.0);
        assert_eq!(keys[0].max.y, 54.0);
    }
}
