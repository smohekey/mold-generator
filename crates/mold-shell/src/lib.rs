use std::num::NonZeroUsize;

use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::{Bounds3, SolidKernel, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellSettings {
    /// Thickness of the cavity-following mold skin.
    pub thickness: f64,
    /// Extra width of flat split/section webs beyond the source part bounds.
    pub flange_width: f64,
    /// Thickness of flat split and section webs.
    pub web_thickness: f64,
}

impl Default for ShellSettings {
    fn default() -> Self {
        Self {
            thickness: 3.0,
            flange_width: 12.0,
            web_thickness: 3.0,
        }
    }
}

pub fn generate_sectioned_shell_mold<K>(
    kernel: &K,
    part: &K::Solid,
    split_axis: Axis,
    section_axis: Axis,
    section_count: NonZeroUsize,
    settings: ShellSettings,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let part_bounds = kernel.bounds(part)?;
    let expanded = kernel.offset(part, settings.thickness)?;
    let skin = kernel.difference(&expanded, part)?;
    let expanded_bounds = kernel.bounds(&expanded)?;
    let split = midpoint(part_bounds, split_axis);
    let (negative_half, positive_half) = split_bounds(expanded_bounds, split_axis, split);
    let sections = section_ranges(expanded_bounds, section_axis, section_count);

    let negative = generate_half(
        kernel,
        part,
        &skin,
        part_bounds,
        negative_half,
        split_axis,
        section_axis,
        split,
        &sections,
        settings,
        Half::Negative,
    )?;
    let positive = generate_half(
        kernel,
        part,
        &skin,
        part_bounds,
        positive_half,
        split_axis,
        section_axis,
        split,
        &sections,
        settings,
        Half::Positive,
    )?;

    Ok(SectionedTwoPartMold { negative, positive })
}

#[derive(Debug, Clone, Copy)]
enum Half {
    Negative,
    Positive,
}

#[allow(clippy::too_many_arguments)]
fn generate_half<K>(
    kernel: &K,
    part: &K::Solid,
    skin: &K::Solid,
    part_bounds: Bounds3,
    half_bounds: Bounds3,
    split_axis: Axis,
    section_axis: Axis,
    split: f64,
    section_ranges: &[(f64, f64)],
    settings: ShellSettings,
    half: Half,
) -> Result<Vec<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let mut pieces = Vec::with_capacity(section_ranges.len());

    for &(section_min, section_max) in section_ranges {
        let mut clip = half_bounds;
        set_min(&mut clip, section_axis, section_min);
        set_max(&mut clip, section_axis, section_max);
        let clip_solid = kernel.cuboid(clip)?;
        let mut piece = kernel.intersection(skin, &clip_solid)?;

        let split_web = split_web_bounds(
            part_bounds,
            clip,
            split_axis,
            section_axis,
            split,
            settings,
            half,
        );
        piece = union_web(kernel, part, piece, split_web)?;

        let min_web = section_web_bounds(
            part_bounds,
            clip,
            split_axis,
            section_axis,
            section_min,
            settings,
            half,
            WebSide::Min,
        );
        piece = union_web(kernel, part, piece, min_web)?;

        let max_web = section_web_bounds(
            part_bounds,
            clip,
            split_axis,
            section_axis,
            section_max,
            settings,
            half,
            WebSide::Max,
        );
        piece = union_web(kernel, part, piece, max_web)?;

        pieces.push(piece);
    }

    Ok(pieces)
}

fn union_web<K>(
    kernel: &K,
    part: &K::Solid,
    piece: K::Solid,
    bounds: Bounds3,
) -> Result<K::Solid, K::Error>
where
    K: SolidKernel,
{
    let web = kernel.cuboid(bounds)?;
    let web = kernel.difference(&web, part)?;
    kernel.union(&piece, &web)
}

fn split_web_bounds(
    part_bounds: Bounds3,
    section_bounds: Bounds3,
    split_axis: Axis,
    section_axis: Axis,
    split: f64,
    settings: ShellSettings,
    half: Half,
) -> Bounds3 {
    let mut bounds = padded(part_bounds, settings.flange_width);
    copy_axis_range(&mut bounds, section_bounds, section_axis);

    match half {
        Half::Negative => {
            set_min(&mut bounds, split_axis, split - settings.web_thickness);
            set_max(&mut bounds, split_axis, split);
        }
        Half::Positive => {
            set_min(&mut bounds, split_axis, split);
            set_max(&mut bounds, split_axis, split + settings.web_thickness);
        }
    }

    bounds
}

#[derive(Debug, Clone, Copy)]
enum WebSide {
    Min,
    Max,
}

fn section_web_bounds(
    part_bounds: Bounds3,
    section_bounds: Bounds3,
    split_axis: Axis,
    section_axis: Axis,
    interface: f64,
    settings: ShellSettings,
    half: Half,
    side: WebSide,
) -> Bounds3 {
    let mut bounds = padded(part_bounds, settings.flange_width);

    let (half_min, half_max) = axis_range(section_bounds, split_axis);
    match half {
        Half::Negative => {
            set_min(&mut bounds, split_axis, half_min);
            set_max(&mut bounds, split_axis, half_max);
        }
        Half::Positive => {
            set_min(&mut bounds, split_axis, half_min);
            set_max(&mut bounds, split_axis, half_max);
        }
    }

    match side {
        WebSide::Min => {
            set_min(&mut bounds, section_axis, interface);
            set_max(
                &mut bounds,
                section_axis,
                interface + settings.web_thickness,
            );
        }
        WebSide::Max => {
            set_min(
                &mut bounds,
                section_axis,
                interface - settings.web_thickness,
            );
            set_max(&mut bounds, section_axis, interface);
        }
    }

    bounds
}

fn padded(bounds: Bounds3, amount: f64) -> Bounds3 {
    Bounds3 {
        min: Vec3::new(
            bounds.min.x - amount,
            bounds.min.y - amount,
            bounds.min.z - amount,
        ),
        max: Vec3::new(
            bounds.max.x + amount,
            bounds.max.y + amount,
            bounds.max.z + amount,
        ),
    }
}

fn midpoint(bounds: Bounds3, axis: Axis) -> f64 {
    let (min, max) = axis_range(bounds, axis);
    (min + max) * 0.5
}

fn split_bounds(bounds: Bounds3, axis: Axis, split: f64) -> (Bounds3, Bounds3) {
    let mut negative = bounds;
    let mut positive = bounds;
    set_max(&mut negative, axis, split);
    set_min(&mut positive, axis, split);
    (negative, positive)
}

fn section_ranges(
    bounds: Bounds3,
    axis: Axis,
    count: NonZeroUsize,
) -> Vec<(f64, f64)> {
    let (min, max) = axis_range(bounds, axis);
    let width = (max - min) / count.get() as f64;

    (0..count.get())
        .map(|index| {
            let start = min + index as f64 * width;
            let end = if index + 1 == count.get() {
                max
            } else {
                min + (index + 1) as f64 * width
            };
            (start, end)
        })
        .collect()
}

fn copy_axis_range(target: &mut Bounds3, source: Bounds3, axis: Axis) {
    let (min, max) = axis_range(source, axis);
    set_min(target, axis, min);
    set_max(target, axis, max);
}

fn axis_range(bounds: Bounds3, axis: Axis) -> (f64, f64) {
    match axis {
        Axis::X => (bounds.min.x, bounds.max.x),
        Axis::Y => (bounds.min.y, bounds.max.y),
        Axis::Z => (bounds.min.z, bounds.max.z),
    }
}

fn set_min(bounds: &mut Bounds3, axis: Axis, value: f64) {
    match axis {
        Axis::X => bounds.min.x = value,
        Axis::Y => bounds.min.y = value,
        Axis::Z => bounds.min.z = value,
    }
}

fn set_max(bounds: &mut Bounds3, axis: Axis, value: f64) {
    match axis {
        Axis::X => bounds.max.x = value,
        Axis::Y => bounds.max.y = value,
        Axis::Z => bounds.max.z = value,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mold_manifold::ManifoldKernel;

    #[test]
    fn cuboid_shell_is_thinner_than_a_solid_block() {
        let kernel = ManifoldKernel;
        let part = kernel
            .cuboid(Bounds3 {
                min: Vec3::new(-20.0, -50.0, -5.0),
                max: Vec3::new(20.0, 50.0, 5.0),
            })
            .unwrap();

        let mold = generate_sectioned_shell_mold(
            &kernel,
            &part,
            Axis::Z,
            Axis::Y,
            NonZeroUsize::new(2).unwrap(),
            ShellSettings::default(),
        )
        .unwrap();

        assert_eq!(mold.negative.len(), 2);
        assert_eq!(mold.positive.len(), 2);
        for piece in mold.negative.iter().chain(&mold.positive) {
            assert_eq!(piece.0.status().to_str(), "No Error");
            assert!(!piece.0.is_empty());
            assert!(piece.0.volume() > 0.0);
        }
    }
}
