use std::{fs, path::Path};

use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::SolidKernel;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{
    FlangeDivisionSettings, FlangeEdge, PartingRegions, PrintTile, PrintVolume, SegmentBoundary,
    SegmentFlangeSettings, SegmentationSettings, ShellSettings, TiledSegmentationSettings,
    WebbingSettings, alternating_rib_layouts, attach_structural_webbing, divide_flange,
    generate_sectioned_shell_mold_with_parting_ranges, partition_tiles_for_print_volume,
    split_with_cumulative_cutters,
};
use mold_test_models::{
    PrintableEnvelope, RibPathSpec, WingEdge, WingSpec, WingSurface, chord_band_region,
    chord_region_extended, printable_tile_dimensions, registration_diamond,
    sample_longitudinal_surface_path, sample_rib_surface_path, segment_normal,
    transverse_flange_blank, wing_segment_boundaries,
};

const FLANGE_MARGIN: f64 = 12.0;
const RIB_SAMPLES: usize = 24;
const CANDIDATE_STEP: f64 = 25.0;
const MODEL_SCALE: f64 = 1.2;
const SHELL_THICKNESS: f64 = 3.0;

#[derive(Debug, Clone, Copy)]
struct FixtureSettings {
    chord_half_width: f64,
    span_half_width: f64,
    normal_half_depth: f64,
}

#[derive(Debug, Clone, Copy)]
struct RegistrationSettings {
    leading: FixtureSettings,
    trailing: FixtureSettings,
    division: FlangeDivisionSettings,
}

impl Default for RegistrationSettings {
    fn default() -> Self {
        let fixture = FixtureSettings {
            chord_half_width: 5.0,
            span_half_width: 7.0,
            normal_half_depth: 2.25,
        };
        Self {
            leading: fixture,
            trailing: fixture,
            division: FlangeDivisionSettings {
                fixture_span: fixture.span_half_width * 2.0,
                max_spacing_ratio: 10.0,
                edge_margin_ratio: 1.5,
                minimum_per_segment: 2,
            },
        }
    }
}

struct RegistrationInsert {
    name: String,
    solid: ManifoldSolid,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = Path::new("target/sample-mold");
    fs::create_dir_all(output)?;

    let mut spec = mold_test_models::preset("gull")?;
    spec.profile_points = 24;
    for station in &mut spec.stations {
        station.span *= MODEL_SCALE;
        station.chord *= MODEL_SCALE;
        station.x_offset *= MODEL_SCALE;
        station.z_offset *= MODEL_SCALE;
    }
    let kernel = ManifoldKernel;
    let part = ManifoldSolid(mold_test_models::generate(&spec)?);
    mold_test_models::write_stl(&part.0, output.join("gull-wing.stl"))?;

    let lower_region = ManifoldSolid(chord_region_extended(
        &spec,
        -500.0,
        0.0,
        80.0,
        SHELL_THICKNESS,
    )?);
    let upper_region = ManifoldSolid(chord_region_extended(
        &spec,
        0.0,
        500.0,
        80.0,
        SHELL_THICKNESS,
    )?);
    let lower_flange = ManifoldSolid(chord_region_extended(
        &spec,
        -3.0,
        0.0,
        FLANGE_MARGIN,
        SHELL_THICKNESS,
    )?);
    let upper_flange = ManifoldSolid(chord_region_extended(
        &spec,
        0.0,
        3.0,
        FLANGE_MARGIN,
        SHELL_THICKNESS,
    )?);

    let shell_settings = ShellSettings {
        thickness: SHELL_THICKNESS,
        structural_webbing: None,
        ..Default::default()
    };
    let webbing = WebbingSettings::default();
    let segment_flanges = SegmentFlangeSettings::default();
    let expanded_bounds = kernel.bounds(&kernel.offset(&part, shell_settings.thickness)?)?;
    let segmentation = SegmentationSettings {
        print_volume: PrintVolume {
            width: 256.0,
            depth: 256.0,
            height: 256.0,
            clearance: 6.0,
        },
        preferred_segment_count: None,
        max_segment_count: 8,
    };
    let candidates = wing_segment_boundaries(
        &spec,
        expanded_bounds.min.y,
        expanded_bounds.max.y,
        CANDIDATE_STEP,
    )?;
    let envelope = PrintableEnvelope {
        flange_margin: FLANGE_MARGIN,
        shell_thickness: shell_settings.thickness,
        web_depth: webbing.depth.max(segment_flanges.width),
        span_samples: 24,
    };
    let boundaries: Vec<SegmentBoundary> = candidates
        .iter()
        .map(|candidate| SegmentBoundary {
            position: candidate.position,
            preference: candidate.deviation,
        })
        .collect();
    let tiles = partition_tiles_for_print_volume(
        &boundaries,
        TiledSegmentationSettings {
            span: segmentation,
            max_longitudinal_segments: 4,
        },
        |start, end, chord| printable_tile_dimensions(&spec, start, end, chord, envelope).ok(),
    )?;
    let mut ranges = Vec::new();
    for tile in &tiles {
        if ranges.last() != Some(&tile.span) {
            ranges.push(tile.span);
        }
    }
    println!(
        "print-volume-aware segment ranges for {:?}: {ranges:?}",
        segmentation.print_volume
    );
    println!("print tiles: {tiles:?}");
    for (index, tile) in tiles.iter().enumerate() {
        println!(
            "tile {} print dimensions: {:?}",
            index + 1,
            printable_tile_dimensions(&spec, tile.span.0, tile.span.1, tile.chord, envelope)?
        );
    }

    let registration = RegistrationSettings::default();
    let inserts = build_registration_inserts(&spec, &ranges, registration)?;
    let socket_cutters: Vec<&ManifoldSolid> = inserts.iter().map(|insert| &insert.solid).collect();
    let (lower_ribs, upper_ribs) = build_ribs(&kernel, &spec, &ranges, registration, webbing)?;

    let mut baseline = generate_sectioned_shell_mold_with_parting_ranges(
        &kernel,
        &part,
        PartingRegions {
            negative: &lower_region,
            positive: &upper_region,
            negative_flange: Some(&lower_flange),
            positive_flange: Some(&upper_flange),
            negative_sockets: &socket_cutters,
            positive_sockets: &socket_cutters,
            negative_webbing_exclusions: &[],
            positive_webbing_exclusions: &[],
        },
        Axis::Y,
        &ranges,
        shell_settings,
    )?;
    add_segment_join_flanges(
        &kernel,
        &spec,
        &part,
        &lower_region,
        &upper_region,
        &mut baseline,
        &ranges,
        &tiles,
        segment_flanges,
        shell_settings.thickness,
        &socket_cutters,
    )?;
    let mut mold = SectionedTwoPartMold {
        negative: baseline.negative.clone(),
        positive: baseline.positive.clone(),
    };
    attach_structural_webbing(
        &kernel,
        &part,
        &mut mold.negative,
        &lower_ribs,
        &socket_cutters,
    )?;
    attach_structural_webbing(
        &kernel,
        &part,
        &mut mold.positive,
        &upper_ribs,
        &socket_cutters,
    )?;
    let baseline = split_mold_into_tiles(&kernel, &spec, baseline, &ranges, &tiles)?;
    let mold = split_mold_into_tiles(&kernel, &spec, mold, &ranges, &tiles)?;
    validate_no_part_intrusion(&part, &mold)?;
    validate_root_offset(&kernel, &part, &mold, shell_settings.thickness)?;

    let lower_webbing = additions(&kernel, &mold.negative, &baseline.negative)?;
    let upper_webbing = additions(&kernel, &mold.positive, &baseline.positive)?;
    validate_attached("lower", &mold.negative, &baseline.negative, &lower_webbing)?;
    validate_attached("upper", &mold.positive, &baseline.positive, &upper_webbing)?;
    export_artifacts(&kernel, output, &part, &mold, &inserts)?;
    Ok(())
}

fn validate_root_offset(
    kernel: &ManifoldKernel,
    part: &ManifoldSolid,
    mold: &SectionedTwoPartMold<ManifoldSolid>,
    expected_offset: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let wing_root = kernel.bounds(part)?.min.y;
    let mold_root = mold
        .negative
        .iter()
        .chain(&mold.positive)
        .map(|piece| kernel.bounds(piece).map(|bounds| bounds.min.y))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    if mold_root > wing_root - expected_offset * 0.9 {
        return Err(format!(
            "root mold face is not offset from wing: wing={wing_root:.6}, mold={mold_root:.6}"
        )
        .into());
    }
    Ok(())
}

fn validate_no_part_intrusion(
    part: &ManifoldSolid,
    mold: &SectionedTwoPartMold<ManifoldSolid>,
) -> Result<(), Box<dyn std::error::Error>> {
    for (half, pieces) in [("lower", &mold.negative), ("upper", &mold.positive)] {
        for (index, piece) in pieces.iter().enumerate() {
            let intrusion = piece.0.intersection(&part.0).volume();
            if intrusion > 1.0e-6 {
                return Err(format!(
                    "{half} tile {} intrudes into the wing by {intrusion:.9}",
                    index + 1
                )
                .into());
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn add_segment_join_flanges(
    kernel: &ManifoldKernel,
    spec: &WingSpec,
    part: &ManifoldSolid,
    lower_region: &ManifoldSolid,
    upper_region: &ManifoldSolid,
    mold: &mut SectionedTwoPartMold<ManifoldSolid>,
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
    settings: SegmentFlangeSettings,
    shell_thickness: f64,
    exclusions: &[&ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    if !settings.width.is_finite()
        || settings.width <= shell_thickness
        || settings.axial_thickness <= 0.0
        || !(0.0..90.0).contains(&settings.maximum_overhang_angle_deg)
    {
        return Err("invalid segment flange settings".into());
    }
    let model_start = spec.stations.first().ok_or("wing has no stations")?.span;
    let model_end = spec.stations.last().ok_or("wing has no stations")?.span;

    let attach = |piece: &mut ManifoldSolid,
                  blank: ManifoldSolid,
                  half_region: &ManifoldSolid|
     -> Result<(), Box<dyn std::error::Error>> {
        let external = kernel.difference(&blank, part)?;
        let half = kernel.intersection(&external, half_region)?;
        *piece = kernel.union_attached(piece, &half)?;
        Ok(())
    };

    for seam in 0..ranges.len().saturating_sub(1) {
        let span = ranges[seam].1;
        let ramp = settings.top_ramp_length();
        let top_blank = ManifoldSolid(transverse_flange_blank(
            spec,
            &[(span - ramp, shell_thickness), (span, settings.width)],
        )?);
        let bed_blank = ManifoldSolid(transverse_flange_blank(
            spec,
            &[
                (span, settings.width),
                (span + settings.axial_thickness, settings.width),
            ],
        )?);
        for (pieces, region) in [
            (&mut mold.negative, lower_region),
            (&mut mold.positive, upper_region),
        ] {
            attach(&mut pieces[seam], top_blank.clone(), region)?;
            attach(&mut pieces[seam + 1], bed_blank.clone(), region)?;
        }
    }

    for (section, range) in ranges.iter().enumerate() {
        let section_tiles: Vec<&PrintTile> =
            tiles.iter().filter(|tile| tile.span == *range).collect();
        if section_tiles.len() < 2 {
            continue;
        }
        for pair in section_tiles.windows(2) {
            let chord_fraction = pair[0].chord.1;
            let start = range.0.max(model_start);
            let end = range.1.min(model_end);
            let upper_direction = segment_normal(spec, start, end)?;
            let lower_direction = upper_direction.map(|value| -value);
            let make_flange = |surface,
                               direction|
             -> Result<ManifoldSolid, Box<dyn std::error::Error>> {
                let path = sample_longitudinal_surface_path(
                    spec,
                    chord_fraction,
                    start,
                    end,
                    RIB_SAMPLES,
                    surface,
                )?;
                let path: Vec<[f64; 3]> = path
                    .into_iter()
                    .map(|point| add_scaled(point, direction, shell_thickness * 0.8))
                    .collect();
                Ok(kernel.swept_rib(&path, direction, settings.axial_thickness, settings.width)?)
            };
            let lower = make_flange(WingSurface::Lower, lower_direction)?;
            let upper = make_flange(WingSurface::Upper, upper_direction)?;
            mold.negative[section] = kernel.union_attached(&mold.negative[section], &lower)?;
            mold.positive[section] = kernel.union_attached(&mold.positive[section], &upper)?;
        }
    }
    for piece in mold.negative.iter_mut().chain(&mut mold.positive) {
        for exclusion in exclusions {
            *piece = kernel.difference(piece, exclusion)?;
        }
    }
    Ok(())
}

fn add_scaled(point: [f64; 3], direction: [f64; 3], distance: f64) -> [f64; 3] {
    [
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
        point[2] + direction[2] * distance,
    ]
}

fn split_mold_into_tiles(
    kernel: &ManifoldKernel,
    spec: &WingSpec,
    mold: SectionedTwoPartMold<ManifoldSolid>,
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
) -> Result<SectionedTwoPartMold<ManifoldSolid>, Box<dyn std::error::Error>> {
    let split_half =
        |pieces: &[ManifoldSolid]| -> Result<Vec<ManifoldSolid>, Box<dyn std::error::Error>> {
            let mut split = Vec::with_capacity(tiles.len());
            for (section, range) in ranges.iter().enumerate() {
                let first_output = split.len();
                let section_tiles: Vec<&PrintTile> =
                    tiles.iter().filter(|tile| tile.span == *range).collect();
                if section_tiles.is_empty() {
                    return Err("span section has no print tiles".into());
                }
                let mut cutters = Vec::with_capacity(section_tiles.len().saturating_sub(1));
                for tile in section_tiles.iter().take(section_tiles.len() - 1) {
                    cutters.push(ManifoldSolid(chord_band_region(
                        spec,
                        (0.0, tile.chord.1),
                        -1_000.0,
                        1_000.0,
                        100.0,
                    )?));
                }
                split.extend(split_with_cumulative_cutters(
                    kernel,
                    &pieces[section],
                    &cutters,
                )?);
                validate_tile_partition(&pieces[section], &split[first_output..])?;
            }
            Ok(split)
        };
    Ok(SectionedTwoPartMold {
        negative: split_half(&mold.negative)?,
        positive: split_half(&mold.positive)?,
    })
}

fn validate_tile_partition(
    original: &ManifoldSolid,
    tiles: &[ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    let mut rejoined = tiles[0].0.clone();
    for tile in &tiles[1..] {
        rejoined = rejoined.union(&tile.0);
    }
    let missing = original.0.difference(&rejoined).volume();
    let excess = rejoined.difference(&original.0).volume();
    if missing > 1.0e-6 || excess > 1.0e-6 {
        return Err(format!(
            "tile split changed mold volume: missing={missing:.9}, excess={excess:.9}"
        )
        .into());
    }
    Ok(())
}

fn build_registration_inserts(
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    settings: RegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    let mut inserts = Vec::new();
    for (segment, &range) in ranges.iter().enumerate() {
        for edge in [FlangeEdge::Leading, FlangeEdge::Trailing] {
            let fixture = match edge {
                FlangeEdge::Leading => settings.leading,
                FlangeEdge::Trailing => settings.trailing,
            };
            for (index, span) in divide_flange(range, settings.division)
                .into_iter()
                .enumerate()
            {
                let side = edge_name(edge);
                inserts.push(RegistrationInsert {
                    name: format!(
                        "registration-insert-s{:02}-{side}-{:02}",
                        segment + 1,
                        index + 1
                    ),
                    solid: ManifoldSolid(registration_diamond(
                        spec,
                        wing_edge(edge),
                        span,
                        fixture.chord_half_width,
                        fixture.span_half_width,
                        fixture.normal_half_depth,
                    )?),
                });
            }
        }
    }
    println!("placed {} registration inserts", inserts.len());
    Ok(inserts)
}

type SegmentRibs = Vec<Vec<ManifoldSolid>>;

fn build_ribs(
    kernel: &ManifoldKernel,
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    registration: RegistrationSettings,
    webbing: WebbingSettings,
) -> Result<(SegmentRibs, SegmentRibs), Box<dyn std::error::Error>> {
    let mut lower = Vec::new();
    let mut upper = Vec::new();
    for &range in ranges {
        let fixtures = divide_flange(range, registration.division);
        let layouts = alternating_rib_layouts(&fixtures, &fixtures);
        let upper_direction = segment_normal(spec, range.0, range.1)?;
        let lower_direction = upper_direction.map(|value| -value);
        let mut lower_segment = Vec::new();
        let mut upper_segment = Vec::new();
        for layout in layouts {
            let lower_path = rib_path(spec, layout, webbing, WingSurface::Lower)?;
            let upper_path = rib_path(spec, layout, webbing, WingSurface::Upper)?;
            lower_segment.push(kernel.swept_rib(
                &lower_path,
                lower_direction,
                webbing.thickness(),
                webbing.depth,
            )?);
            upper_segment.push(kernel.swept_rib(
                &upper_path,
                upper_direction,
                webbing.thickness(),
                webbing.depth,
            )?);
        }
        lower.push(lower_segment);
        upper.push(upper_segment);
    }
    Ok((lower, upper))
}

fn rib_path(
    spec: &WingSpec,
    layout: mold_shell::RibLayout,
    webbing: WebbingSettings,
    surface: WingSurface,
) -> Result<Vec<[f64; 3]>, Box<dyn std::error::Error>> {
    Ok(sample_rib_surface_path(
        spec,
        RibPathSpec {
            start_edge: wing_edge(layout.start.edge),
            start_span: layout.start.span,
            end_edge: wing_edge(layout.end.edge),
            end_span: layout.end.span,
            flange_margin: FLANGE_MARGIN,
            rib_thickness: webbing.thickness(),
            samples: RIB_SAMPLES,
            surface,
        },
    )?)
}

fn additions(
    kernel: &ManifoldKernel,
    webbed: &[ManifoldSolid],
    baseline: &[ManifoldSolid],
) -> Result<Vec<ManifoldSolid>, mold_manifold::ManifoldKernelError> {
    webbed
        .iter()
        .zip(baseline)
        .map(|(piece, plain)| kernel.difference(piece, plain))
        .collect()
}

fn validate_attached(
    half: &str,
    webbed: &[ManifoldSolid],
    baseline: &[ManifoldSolid],
    additions: &[ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    for (index, ((piece, plain), added)) in webbed.iter().zip(baseline).zip(additions).enumerate() {
        if piece.0.decompose().len() > plain.0.decompose().len() {
            return Err(format!("{half} segment {} gained a disconnected rib", index + 1).into());
        }
        if added.0.is_empty() || added.0.volume() <= 1.0e-9 {
            return Err(format!("{half} segment {} has no attached webbing", index + 1).into());
        }
    }
    Ok(())
}

fn export_artifacts(
    kernel: &ManifoldKernel,
    output: &Path,
    part: &ManifoldSolid,
    mold: &SectionedTwoPartMold<ManifoldSolid>,
    inserts: &[RegistrationInsert],
) -> Result<(), Box<dyn std::error::Error>> {
    for (prefix, pieces) in [("lower", &mold.negative), ("upper", &mold.positive)] {
        for (index, piece) in pieces.iter().enumerate() {
            kernel.export_stl(
                piece,
                output.join(format!("mold-{prefix}-{:02}.stl", index + 1)),
            )?;
        }
    }
    for insert in inserts {
        kernel.export_stl(&insert.solid, output.join(format!("{}.stl", insert.name)))?;
    }
    let mut assembly = vec![ThreeMfObject {
        name: "wing".to_owned(),
        solid: part,
    }];
    for (prefix, pieces) in [("lower", &mold.negative), ("upper", &mold.positive)] {
        for (index, piece) in pieces.iter().enumerate() {
            assembly.push(ThreeMfObject {
                name: format!("mold-{prefix}-{:02}", index + 1),
                solid: piece,
            });
        }
    }
    for insert in inserts {
        assembly.push(ThreeMfObject {
            name: insert.name.clone(),
            solid: &insert.solid,
        });
    }
    write_3mf(
        output.join("gull-wing-mold-assembly.3mf"),
        "Gull wing mold validation assembly",
        &assembly,
    )?;
    println!("generated visual sample in {}", output.display());
    Ok(())
}

const fn wing_edge(edge: FlangeEdge) -> WingEdge {
    match edge {
        FlangeEdge::Leading => WingEdge::Leading,
        FlangeEdge::Trailing => WingEdge::Trailing,
    }
}

const fn edge_name(edge: FlangeEdge) -> &'static str {
    match edge {
        FlangeEdge::Leading => "leading",
        FlangeEdge::Trailing => "trailing",
    }
}
