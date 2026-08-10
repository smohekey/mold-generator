use std::num::NonZeroUsize;

use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::{Bounds3, SolidKernel, Transform3, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WebbingSettings {
    /// Axis pointing from the cavity toward the back of a custom-parted half.
    pub back_axis: Axis,
    pub extrusion_width: f64,
    pub wall_line_count: usize,
    pub longitudinal_web_count: usize,
    pub max_brace_spacing: f64,
    pub depth: f64,
    pub exclusion_clearance: f64,
}

impl WebbingSettings {
    pub fn thickness(self) -> f64 {
        self.extrusion_width * self.wall_line_count as f64
    }
}

impl Default for WebbingSettings {
    fn default() -> Self {
        Self {
            back_axis: Axis::Z,
            extrusion_width: 0.45,
            wall_line_count: 3,
            longitudinal_web_count: 3,
            max_brace_spacing: 45.0,
            depth: 8.0,
            exclusion_clearance: 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellSettings {
    pub thickness: f64,
    pub flange_width: f64,
    pub web_thickness: f64,
    pub structural_webbing: Option<WebbingSettings>,
}

impl Default for ShellSettings {
    fn default() -> Self {
        Self {
            thickness: 3.0,
            flange_width: 12.0,
            web_thickness: 3.0,
            structural_webbing: Some(WebbingSettings::default()),
        }
    }
}

/// Explicit geometry used to divide a shell into its two mold halves.
/// Registration sockets are cut from the final printable pieces so later
/// clipping or construction steps cannot refill or remove them.
pub struct PartingRegions<'a, S> {
    pub negative: &'a S,
    pub positive: &'a S,
    pub negative_flange: Option<&'a S>,
    pub positive_flange: Option<&'a S>,
    pub negative_sockets: &'a [&'a S],
    pub positive_sockets: &'a [&'a S],
    /// Reserved volumes that structural webbing must not enter. Registration
    /// socket cutters are automatically treated as exclusions too.
    pub negative_webbing_exclusions: &'a [&'a S],
    pub positive_webbing_exclusions: &'a [&'a S],
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
    let mut negative = generate_half(
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
    let mut positive = generate_half(
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
    if let Some(webbing) = settings.structural_webbing {
        add_structural_webbing(
            kernel,
            part,
            &mut negative,
            section_axis,
            split_axis,
            Half::Negative,
            webbing,
            std::iter::empty(),
        )?;
        add_structural_webbing(
            kernel,
            part,
            &mut positive,
            section_axis,
            split_axis,
            Half::Positive,
            webbing,
            std::iter::empty(),
        )?;
    }
    Ok(SectionedTwoPartMold { negative, positive })
}

pub fn generate_sectioned_shell_mold_with_parting<K>(
    kernel: &K,
    part: &K::Solid,
    parting: PartingRegions<'_, K::Solid>,
    section_axis: Axis,
    section_count: NonZeroUsize,
    settings: ShellSettings,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let expanded = kernel.offset(part, settings.thickness)?;
    let expanded_bounds = kernel.bounds(&expanded)?;
    let sections = section_ranges(expanded_bounds, section_axis, section_count);
    generate_sectioned_shell_mold_with_parting_ranges(
        kernel,
        part,
        parting,
        section_axis,
        &sections,
        settings,
    )
}

pub fn generate_sectioned_shell_mold_with_parting_ranges<K>(
    kernel: &K,
    part: &K::Solid,
    parting: PartingRegions<'_, K::Solid>,
    section_axis: Axis,
    sections: &[(f64, f64)],
    settings: ShellSettings,
) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let expanded = kernel.offset(part, settings.thickness)?;
    let skin = kernel.difference(&expanded, part)?;
    let mut negative_skin = kernel.intersection(&skin, parting.negative)?;
    let mut positive_skin = kernel.intersection(&skin, parting.positive)?;
    if let Some(flange) = parting.negative_flange {
        negative_skin = kernel.union(&negative_skin, &kernel.difference(flange, part)?)?;
    }
    if let Some(flange) = parting.positive_flange {
        positive_skin = kernel.union(&positive_skin, &kernel.difference(flange, part)?)?;
    }

    // Determine clipping bounds from the complete mold halves, including the
    // flange extensions rather than only the offset source part.
    let negative_bounds = kernel.bounds(&negative_skin)?;
    let positive_bounds = kernel.bounds(&positive_skin)?;
    let mold_bounds = union_bounds(negative_bounds, positive_bounds);

    let mut negative = clip_sections(kernel, &negative_skin, mold_bounds, section_axis, sections)?;
    let mut positive = clip_sections(kernel, &positive_skin, mold_bounds, section_axis, sections)?;

    if let Some(webbing) = settings.structural_webbing {
        add_structural_webbing(
            kernel,
            part,
            &mut negative,
            section_axis,
            webbing.back_axis,
            Half::Negative,
            webbing,
            parting
                .negative_sockets
                .iter()
                .copied()
                .chain(parting.negative_webbing_exclusions.iter().copied()),
        )?;
        add_structural_webbing(
            kernel,
            part,
            &mut positive,
            section_axis,
            webbing.back_axis,
            Half::Positive,
            webbing,
            parting
                .positive_sockets
                .iter()
                .copied()
                .chain(parting.positive_webbing_exclusions.iter().copied()),
        )?;
    }

    // Registration is deliberately the final geometry operation. This makes
    // each supplied cutter authoritative: if it intersects a printable piece,
    // that exact volume is removed and nothing downstream can refill it.
    subtract_cutters(kernel, &mut negative, parting.negative_sockets)?;
    subtract_cutters(kernel, &mut positive, parting.positive_sockets)?;

    Ok(SectionedTwoPartMold { negative, positive })
}

fn subtract_cutters<K>(
    kernel: &K,
    pieces: &mut [K::Solid],
    cutters: &[&K::Solid],
) -> Result<(), K::Error>
where
    K: SolidKernel,
{
    for piece in pieces {
        for cutter in cutters {
            *piece = kernel.difference(piece, cutter)?;
        }
    }
    Ok(())
}

fn clip_sections<K>(
    kernel: &K,
    solid: &K::Solid,
    bounds: Bounds3,
    section_axis: Axis,
    sections: &[(f64, f64)],
) -> Result<Vec<K::Solid>, K::Error>
where
    K: SolidKernel,
{
    let mut pieces = Vec::with_capacity(sections.len());
    for &(section_min, section_max) in sections {
        let mut clip = bounds;
        set_min(&mut clip, section_axis, section_min);
        set_max(&mut clip, section_axis, section_max);
        pieces.push(kernel.intersection(solid, &kernel.cuboid(clip)?)?);
    }
    Ok(pieces)
}

fn union_bounds(a: Bounds3, b: Bounds3) -> Bounds3 {
    Bounds3 {
        min: Vec3::new(
            a.min.x.min(b.min.x),
            a.min.y.min(b.min.y),
            a.min.z.min(b.min.z),
        ),
        max: Vec3::new(
            a.max.x.max(b.max.x),
            a.max.y.max(b.max.y),
            a.max.z.max(b.max.z),
        ),
    }
}

#[derive(Debug, Clone, Copy)]
enum Half {
    Negative,
    Positive,
}

#[allow(clippy::too_many_arguments)]
fn add_structural_webbing<'a, K, I>(
    kernel: &K,
    part: &K::Solid,
    pieces: &mut [K::Solid],
    print_axis: Axis,
    back_axis: Axis,
    half: Half,
    settings: WebbingSettings,
    exclusions: I,
) -> Result<(), K::Error>
where
    K: SolidKernel,
    I: IntoIterator<Item = &'a K::Solid>,
    K::Solid: 'a,
{
    if print_axis == back_axis
        || settings.longitudinal_web_count < 2
        || settings.thickness() <= 0.0
        || settings.depth <= 0.0
    {
        return Ok(());
    }
    let exclusions: Vec<&K::Solid> = exclusions.into_iter().collect();
    let transverse_axis = remaining_axis(print_axis, back_axis);

    for piece in pieces {
        let bounds = kernel.bounds(piece)?;
        let (print_min, print_max) = axis_range(bounds, print_axis);
        let (cross_min, cross_max) = axis_range(bounds, transverse_axis);
        let (back_min, back_max) = axis_range(bounds, back_axis);
        let depth_bounds = match half {
            Half::Negative => (back_min, (back_min + settings.depth).min(back_max)),
            Half::Positive => ((back_max - settings.depth).max(back_min), back_max),
        };
        let count = settings.longitudinal_web_count;
        let inset = (settings.thickness() * 0.5).min((cross_max - cross_min) * 0.25);
        let nodes: Vec<f64> = (0..count)
            .map(|index| {
                cross_min
                    + inset
                    + (cross_max - cross_min - 2.0 * inset) * index as f64 / (count - 1) as f64
            })
            .collect();
        let mut lattice: Option<K::Solid> = None;

        for &cross in &nodes {
            let web = prism_between(
                kernel,
                print_axis,
                transverse_axis,
                back_axis,
                (print_min, cross),
                (print_max, cross),
                depth_bounds,
                settings.thickness(),
            )?;
            lattice = Some(union_optional(kernel, lattice, web)?);
        }

        for lane in 0..nodes.len() - 1 {
            let lane_width = nodes[lane + 1] - nodes[lane];
            let target_step = lane_width.max(settings.thickness());
            let max_step = settings.max_brace_spacing.max(settings.thickness());
            let brace_count = ((print_max - print_min) / target_step.min(max_step))
                .ceil()
                .max(1.0) as usize;
            let step = (print_max - print_min) / brace_count as f64;
            for index in 0..brace_count {
                let a = print_min + index as f64 * step;
                let b = a + step;
                let (cross_a, cross_b) = if (index + lane) % 2 == 0 {
                    (nodes[lane], nodes[lane + 1])
                } else {
                    (nodes[lane + 1], nodes[lane])
                };
                let brace = prism_between(
                    kernel,
                    print_axis,
                    transverse_axis,
                    back_axis,
                    (a, cross_a),
                    (b, cross_b),
                    depth_bounds,
                    settings.thickness(),
                )?;
                lattice = Some(union_optional(kernel, lattice, brace)?);
            }
        }

        if let Some(mut lattice) = lattice {
            lattice = kernel.difference(&lattice, part)?;
            for exclusion in &exclusions {
                let cutter = if settings.exclusion_clearance > 0.0 {
                    kernel.offset(exclusion, settings.exclusion_clearance)?
                } else {
                    (*exclusion).clone()
                };
                lattice = kernel.difference(&lattice, &cutter)?;
            }
            *piece = kernel.union(piece, &lattice)?;
        }
    }
    Ok(())
}

fn union_optional<K: SolidKernel>(
    kernel: &K,
    existing: Option<K::Solid>,
    next: K::Solid,
) -> Result<K::Solid, K::Error> {
    match existing {
        Some(existing) => kernel.union(&existing, &next),
        None => Ok(next),
    }
}

#[allow(clippy::too_many_arguments)]
fn prism_between<K: SolidKernel>(
    kernel: &K,
    print_axis: Axis,
    transverse_axis: Axis,
    back_axis: Axis,
    start: (f64, f64),
    end: (f64, f64),
    depth: (f64, f64),
    thickness: f64,
) -> Result<K::Solid, K::Error> {
    let dp = end.0 - start.0;
    let dc = end.1 - start.1;
    let length = dp.hypot(dc);
    let local = kernel.cuboid(Bounds3 {
        min: Vec3::new(0.0, -thickness * 0.5, depth.0),
        max: Vec3::new(length, thickness * 0.5, depth.1),
    })?;
    let u = (dp / length, dc / length);
    let v = (-u.1, u.0);
    let mut matrix = [[0.0; 4]; 4];
    matrix[3][3] = 1.0;
    set_transform_component(&mut matrix, print_axis, 0, u.0);
    set_transform_component(&mut matrix, transverse_axis, 0, u.1);
    set_transform_component(&mut matrix, print_axis, 1, v.0);
    set_transform_component(&mut matrix, transverse_axis, 1, v.1);
    set_transform_component(&mut matrix, back_axis, 2, 1.0);
    set_transform_component(&mut matrix, print_axis, 3, start.0);
    set_transform_component(&mut matrix, transverse_axis, 3, start.1);
    kernel.transform(&local, Transform3 { matrix })
}

fn set_transform_component(matrix: &mut [[f64; 4]; 4], axis: Axis, column: usize, value: f64) {
    matrix[axis_index(axis)][column] = value;
}

const fn axis_index(axis: Axis) -> usize {
    match axis {
        Axis::X => 0,
        Axis::Y => 1,
        Axis::Z => 2,
    }
}

fn remaining_axis(a: Axis, b: Axis) -> Axis {
    [Axis::X, Axis::Y, Axis::Z]
        .into_iter()
        .find(|axis| *axis != a && *axis != b)
        .expect("two distinct axes always leave one remaining axis")
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
        let mut piece = kernel.intersection(skin, &kernel.cuboid(clip)?)?;
        piece = union_web(
            kernel,
            part,
            piece,
            split_web_bounds(
                part_bounds,
                clip,
                split_axis,
                section_axis,
                split,
                settings,
                half,
            ),
        )?;
        piece = union_web(
            kernel,
            part,
            piece,
            section_web_bounds(
                part_bounds,
                clip,
                split_axis,
                section_axis,
                section_min,
                settings,
                WebSide::Min,
            ),
        )?;
        piece = union_web(
            kernel,
            part,
            piece,
            section_web_bounds(
                part_bounds,
                clip,
                split_axis,
                section_axis,
                section_max,
                settings,
                WebSide::Max,
            ),
        )?;
        pieces.push(piece);
    }
    Ok(pieces)
}

fn union_web<K: SolidKernel>(
    kernel: &K,
    part: &K::Solid,
    piece: K::Solid,
    bounds: Bounds3,
) -> Result<K::Solid, K::Error> {
    let web = kernel.difference(&kernel.cuboid(bounds)?, part)?;
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
    let mut b = padded(part_bounds, settings.flange_width);
    copy_axis_range(&mut b, section_bounds, section_axis);
    match half {
        Half::Negative => {
            set_min(&mut b, split_axis, split - settings.web_thickness);
            set_max(&mut b, split_axis, split)
        }
        Half::Positive => {
            set_min(&mut b, split_axis, split);
            set_max(&mut b, split_axis, split + settings.web_thickness)
        }
    }
    b
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
    side: WebSide,
) -> Bounds3 {
    let mut b = padded(part_bounds, settings.flange_width);
    let (hmin, hmax) = axis_range(section_bounds, split_axis);
    set_min(&mut b, split_axis, hmin);
    set_max(&mut b, split_axis, hmax);
    match side {
        WebSide::Min => {
            set_min(&mut b, section_axis, interface);
            set_max(&mut b, section_axis, interface + settings.web_thickness)
        }
        WebSide::Max => {
            set_min(&mut b, section_axis, interface - settings.web_thickness);
            set_max(&mut b, section_axis, interface)
        }
    }
    b
}

fn padded(b: Bounds3, a: f64) -> Bounds3 {
    Bounds3 {
        min: Vec3::new(b.min.x - a, b.min.y - a, b.min.z - a),
        max: Vec3::new(b.max.x + a, b.max.y + a, b.max.z + a),
    }
}

fn midpoint(b: Bounds3, a: Axis) -> f64 {
    let (m, n) = axis_range(b, a);
    (m + n) * 0.5
}

fn split_bounds(b: Bounds3, a: Axis, s: f64) -> (Bounds3, Bounds3) {
    let mut n = b;
    let mut p = b;
    set_max(&mut n, a, s);
    set_min(&mut p, a, s);
    (n, p)
}

fn section_ranges(b: Bounds3, a: Axis, c: NonZeroUsize) -> Vec<(f64, f64)> {
    let (min, max) = axis_range(b, a);
    let w = (max - min) / c.get() as f64;
    (0..c.get())
        .map(|i| {
            let s = min + i as f64 * w;
            let e = if i + 1 == c.get() {
                max
            } else {
                min + (i + 1) as f64 * w
            };
            (s, e)
        })
        .collect()
}

fn copy_axis_range(t: &mut Bounds3, s: Bounds3, a: Axis) {
    let (min, max) = axis_range(s, a);
    set_min(t, a, min);
    set_max(t, a, max)
}

fn axis_range(b: Bounds3, a: Axis) -> (f64, f64) {
    match a {
        Axis::X => (b.min.x, b.max.x),
        Axis::Y => (b.min.y, b.max.y),
        Axis::Z => (b.min.z, b.max.z),
    }
}

fn set_min(b: &mut Bounds3, a: Axis, v: f64) {
    match a {
        Axis::X => b.min.x = v,
        Axis::Y => b.min.y = v,
        Axis::Z => b.min.z = v,
    }
}

fn set_max(b: &mut Bounds3, a: Axis, v: f64) {
    match a {
        Axis::X => b.max.x = v,
        Axis::Y => b.max.y = v,
        Axis::Z => b.max.z = v,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mold_manifold::ManifoldKernel;

    #[test]
    fn cuboid_shell_is_thinner_than_a_solid_block() {
        let k = ManifoldKernel;
        let p = k
            .cuboid(Bounds3 {
                min: Vec3::new(-20.0, -50.0, -5.0),
                max: Vec3::new(20.0, 50.0, 5.0),
            })
            .unwrap();
        let m = generate_sectioned_shell_mold(
            &k,
            &p,
            Axis::Z,
            Axis::Y,
            NonZeroUsize::new(2).unwrap(),
            ShellSettings::default(),
        )
        .unwrap();
        assert_eq!(m.negative.len(), 2);
        assert_eq!(m.positive.len(), 2);
        for p in m.negative.iter().chain(&m.positive) {
            assert_eq!(p.0.status().to_str(), "No Error");
            assert!(!p.0.is_empty());
            assert!(p.0.volume() > 0.0);
        }
    }

    #[test]
    fn explicit_section_ranges_preserve_a_requested_bend_boundary() {
        let k = ManifoldKernel;
        let part = k
            .cuboid(Bounds3 {
                min: Vec3::new(-20.0, -50.0, -5.0),
                max: Vec3::new(20.0, 50.0, 5.0),
            })
            .unwrap();
        let negative_region = k
            .cuboid(Bounds3 {
                min: Vec3::new(-100.0, -100.0, -100.0),
                max: Vec3::new(100.0, 100.0, 0.0),
            })
            .unwrap();
        let positive_region = k
            .cuboid(Bounds3 {
                min: Vec3::new(-100.0, -100.0, 0.0),
                max: Vec3::new(100.0, 100.0, 100.0),
            })
            .unwrap();
        let empty = [];
        let ranges = [(-53.0, -10.0), (-10.0, 53.0)];
        let mold = generate_sectioned_shell_mold_with_parting_ranges(
            &k,
            &part,
            PartingRegions {
                negative: &negative_region,
                positive: &positive_region,
                negative_flange: None,
                positive_flange: None,
                negative_sockets: &empty,
                positive_sockets: &empty,
                negative_webbing_exclusions: &empty,
                positive_webbing_exclusions: &empty,
            },
            Axis::Y,
            &ranges,
            ShellSettings {
                structural_webbing: None,
                ..Default::default()
            },
        )
        .unwrap();

        for half in [&mold.negative, &mold.positive] {
            assert_eq!(half.len(), 2);
            assert!((k.bounds(&half[0]).unwrap().max.y + 10.0).abs() < 1.0e-9);
            assert!((k.bounds(&half[1]).unwrap().min.y + 10.0).abs() < 1.0e-9);
        }
    }
}
