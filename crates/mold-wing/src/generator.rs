use std::{
    fs,
    path::{Path, PathBuf},
};

use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::SolidKernel;
use mold_manifold::{ManifoldKernel, ManifoldSolid, SweepEndPlane};
use mold_shell::{
    BaseAttachmentGeometry, BaseMoldHalves, FlangeEdge, PartingRegions, PrintTile, PrintVolume,
    SegmentBoundary, SegmentFlangeSettings, SegmentationError, SegmentationSettings, ShellSettings,
    TiledSegmentationSettings, attach_base_sealing_profile,
    generate_sectioned_shell_mold_with_parting_ranges, partition_tiles_for_print_volume,
    split_with_cumulative_cutters,
};
use mold_wing_geometry::{
    FlangeEndObstructions, FlangeFastenerBand, PrintableEnvelope, PrintableFrame,
    WingBaseAttachmentSettings, WingEdge, WingFlangeFastenerSpec,
    WingLongitudinalRegistrationSettings, WingPanelRivetSpec, WingSegmentBoundary, WingSpec,
    WingSurface, WingTransverseRegistrationSettings, chord_region_extended,
    chord_region_with_span_margins, flange_fastener_positions, longitudinal_edge_fastener_cutters,
    longitudinal_split_flange_fastener_cutters, longitudinal_split_flange_registration_inserts,
    panel_rivet_heads, printable_tile_dimensions, printable_tile_frame, registration_diamond,
    sample_longitudinal_surface_path, sampled_chord_band_region, segment_normal,
    transverse_flange_blank, transverse_flange_fastener_cutters,
    transverse_flange_registration_inserts, transverse_section_normal,
    transverse_through_flange_fastener_cutters, wing_base_attachment_geometry,
    wing_segment_boundaries,
};

const FLANGE_MARGIN: f64 = 12.0;
const PARTING_FLANGE_HALF_DEPTH: f64 = 3.0;
const SURFACE_SAMPLES: usize = 24;
const CANDIDATE_STEP: f64 = 25.0;
const MINIMUM_WALL_THICKNESS: f64 = 4.0;
/// Smaller changes describe a smooth loft approximation, not a panel bend
/// worth sacrificing build-volume utilization to preserve as an exact seam.
const MINIMUM_PREFERRED_SEAM_DEVIATION: f64 = 5.0 * std::f64::consts::PI / 180.0;

fn wing_shell_settings(surface_detail_height: f64) -> ShellSettings {
    ShellSettings {
        thickness: MINIMUM_WALL_THICKNESS + surface_detail_height,
        structural_webbing: None,
        ..Default::default()
    }
}

fn planning_boundary(candidate: WingSegmentBoundary) -> SegmentBoundary {
    SegmentBoundary {
        position: candidate.position,
        preference: if candidate.deviation >= MINIMUM_PREFERRED_SEAM_DEVIATION {
            candidate.deviation
        } else {
            0.0
        },
    }
}

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
        }
    }
}

struct RegistrationInsert {
    name: String,
    solid: ManifoldSolid,
    mating: Option<RegistrationMating>,
}

#[derive(Debug, Clone, Copy)]
struct RegistrationMating {
    surface: WingSurface,
    tile_indices: [usize; 2],
}

struct EdgeFastenerHole {
    section: usize,
    edge: FlangeEdge,
    position: f64,
    solid: ManifoldSolid,
}

struct BaseFastenerHole {
    surface: WingSurface,
    chord_fraction: f64,
    solid: ManifoldSolid,
}

struct MoldArtifacts<'a> {
    output: &'a Path,
    part: &'a ManifoldSolid,
    mold: &'a SectionedTwoPartMold<ManifoldSolid>,
    base_sealing_profile: &'a ManifoldSolid,
    inserts: &'a [RegistrationInsert],
    artifact_stem: &'a str,
    assembly_title: &'a str,
}

pub struct WingMoldGenerator {
    spec: WingSpec,
    output: PathBuf,
    artifact_stem: String,
    assembly_title: String,
    model_scale: f64,
    profile_points: usize,
    panel_rivets: Option<WingPanelRivetSpec>,
    flange_fasteners: WingFlangeFastenerSpec,
}

impl WingMoldGenerator {
    pub fn new(
        spec: WingSpec,
        output: impl Into<PathBuf>,
        artifact_stem: impl Into<String>,
        assembly_title: impl Into<String>,
    ) -> Self {
        Self {
            spec,
            output: output.into(),
            artifact_stem: artifact_stem.into(),
            assembly_title: assembly_title.into(),
            model_scale: 1.0,
            profile_points: 24,
            panel_rivets: None,
            flange_fasteners: WingFlangeFastenerSpec::default(),
        }
    }

    pub fn with_model_scale(mut self, model_scale: f64) -> Self {
        self.model_scale = model_scale;
        self
    }

    pub fn with_profile_points(mut self, profile_points: usize) -> Self {
        self.profile_points = profile_points;
        self
    }

    pub fn with_panel_rivets(mut self, panel_rivets: WingPanelRivetSpec) -> Self {
        self.panel_rivets = Some(panel_rivets);
        self
    }

    pub fn with_flange_fasteners(mut self, flange_fasteners: WingFlangeFastenerSpec) -> Self {
        self.flange_fasteners = flange_fasteners;
        self
    }

    pub fn generate(self) -> Result<(), Box<dyn std::error::Error>> {
        generate(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct FinalTileFitFailure {
    tile_index: usize,
    tile: PrintTile,
    half: &'static str,
    dimensions: [f64; 3],
}

fn plan_print_tiles(
    spec: &WingSpec,
    boundaries: &[SegmentBoundary],
    settings: TiledSegmentationSettings,
    envelope: PrintableEnvelope,
    flange_settings: SegmentFlangeSettings,
    fasteners: &WingFlangeFastenerSpec,
    blocked_tiles: &[PrintTile],
) -> Result<Vec<PrintTile>, SegmentationError> {
    let final_span = boundaries
        .last()
        .ok_or(SegmentationError::InvalidSettings(
            "at least two boundaries are required",
        ))?
        .position;
    partition_tiles_for_print_volume(boundaries, settings, |start, end, chord| {
        let tile = PrintTile {
            span: (start, end),
            chord,
        };
        let end_obstructions = longitudinal_flange_end_obstructions(
            (end - final_span).abs() > 1.0e-9,
            flange_settings.top_ramp_length(),
        );
        if blocked_tiles.contains(&tile)
            || flange_fastener_positions((start, end), end_obstructions, fasteners).is_err()
        {
            None
        } else {
            printable_tile_dimensions(spec, start, end, chord, envelope).ok()
        }
    })
}

fn final_tile_fit_failures(
    spec: &WingSpec,
    print_volume: PrintVolume,
    tiles: &[PrintTile],
    mold: &SectionedTwoPartMold<ManifoldSolid>,
) -> Result<Vec<FinalTileFitFailure>, Box<dyn std::error::Error>> {
    if mold.negative.len() != tiles.len() || mold.positive.len() != tiles.len() {
        return Err("final mold piece count does not match the print plan".into());
    }
    let mut failures = Vec::new();
    for (tile_index, tile) in tiles.iter().copied().enumerate() {
        let frame = printable_tile_frame(spec, tile.span.0, tile.span.1)?;
        for (half, pieces) in [("lower", &mold.negative), ("upper", &mold.positive)] {
            let dimensions = solid_dimensions_in_frame(&pieces[tile_index], frame)?;
            if !print_volume.fits(dimensions) {
                failures.push(FinalTileFitFailure {
                    tile_index,
                    tile,
                    half,
                    dimensions,
                });
            }
        }
    }
    Ok(failures)
}

fn solid_dimensions_in_frame(
    solid: &ManifoldSolid,
    frame: PrintableFrame,
) -> Result<[f64; 3], Box<dyn std::error::Error>> {
    let mesh = solid.0.as_original().get_mesh_gl64(-1);
    let stride = mesh.num_prop as usize;
    if stride < 3 || !mesh.vert_properties.len().is_multiple_of(stride) {
        return Err("final mold mesh has invalid vertex properties".into());
    }
    frame
        .dimensions(
            mesh.vert_properties
                .chunks_exact(stride)
                .map(|vertex| [vertex[0], vertex[1], vertex[2]]),
        )
        .ok_or_else(|| "final mold mesh has no finite vertices".into())
}

fn generate(generator: WingMoldGenerator) -> Result<(), Box<dyn std::error::Error>> {
    if !generator.model_scale.is_finite() || generator.model_scale <= 0.0 {
        return Err("model scale must be finite and positive".into());
    }
    let output = generator.output.as_path();
    fs::create_dir_all(output)?;

    let mut spec = generator.spec;
    spec.profile_points = generator.profile_points;
    for station in &mut spec.stations {
        station.span *= generator.model_scale;
        station.chord *= generator.model_scale;
        station.x_offset *= generator.model_scale;
        station.z_offset *= generator.model_scale;
    }
    let panel_rivets = generator
        .panel_rivets
        .map(|rivets| rivets.scaled(generator.model_scale))
        .transpose()?;
    let kernel = ManifoldKernel;
    let base_part = ManifoldSolid(mold_wing_geometry::generate(&spec)?);
    let rivet_heads = panel_rivets
        .as_ref()
        .map(|rivets| panel_rivet_heads(&spec, rivets).map(ManifoldSolid))
        .transpose()?;
    let part = match &rivet_heads {
        Some(heads) => kernel.union_attached(&base_part, heads)?,
        None => base_part.clone(),
    };
    let shell_settings = wing_shell_settings(
        panel_rivets
            .as_ref()
            .map_or(0.0, |rivets| rivets.head_height),
    );

    let lower_region = ManifoldSolid(chord_region_extended(
        &spec,
        -500.0,
        0.0,
        80.0,
        shell_settings.thickness,
    )?);
    let upper_region = ManifoldSolid(chord_region_extended(
        &spec,
        0.0,
        500.0,
        80.0,
        shell_settings.thickness,
    )?);
    let lower_flange = ManifoldSolid(chord_region_with_span_margins(
        &spec,
        -PARTING_FLANGE_HALF_DEPTH,
        0.0,
        FLANGE_MARGIN,
        (0.0, shell_settings.thickness),
    )?);
    let upper_flange = ManifoldSolid(chord_region_with_span_margins(
        &spec,
        0.0,
        PARTING_FLANGE_HALF_DEPTH,
        FLANGE_MARGIN,
        (0.0, shell_settings.thickness),
    )?);

    let segment_flanges = SegmentFlangeSettings::default();
    let root_span = spec.stations.first().ok_or("wing has no stations")?.span;
    let expanded_bounds = kernel.bounds(&kernel.offset(&base_part, shell_settings.thickness)?)?;
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
    let candidates =
        wing_segment_boundaries(&spec, root_span, expanded_bounds.max.y, CANDIDATE_STEP)?;
    let envelope = PrintableEnvelope {
        flange_margin: FLANGE_MARGIN,
        shell_thickness: shell_settings.thickness,
        web_depth: segment_flanges.width,
        span_samples: SURFACE_SAMPLES,
    };
    let boundaries: Vec<SegmentBoundary> =
        candidates.iter().copied().map(planning_boundary).collect();
    let tiled_segmentation = TiledSegmentationSettings {
        span: segmentation,
        max_longitudinal_segments: 4,
    };
    let mut blocked_tiles = Vec::new();
    loop {
        let tiles = plan_print_tiles(
            &spec,
            &boundaries,
            tiled_segmentation,
            envelope,
            segment_flanges,
            &generator.flange_fasteners,
            &blocked_tiles,
        )?;
        let mut ranges = Vec::new();
        for tile in &tiles {
            if ranges.last() != Some(&tile.span) {
                ranges.push(tile.span);
            }
        }
        println!(
            "candidate print-volume-aware segment ranges for {:?}: {ranges:?}",
            segmentation.print_volume
        );
        println!("candidate print tiles: {tiles:?}");
        for (index, tile) in tiles.iter().enumerate() {
            println!(
                "candidate tile {} estimated print dimensions: {:?}",
                index + 1,
                printable_tile_dimensions(&spec, tile.span.0, tile.span.1, tile.chord, envelope)?
            );
        }

        let registration = RegistrationSettings::default();
        let base_geometry = wing_base_attachment_geometry(
            &spec,
            &base_part.0,
            WingBaseAttachmentSettings {
                flange_width: segment_flanges.lateral_flange_margin(shell_settings.thickness),
                axial_thickness: segment_flanges.axial_thickness,
            },
        )?;
        let base_opening = ManifoldSolid(base_geometry.opening);
        let mut base_mold_flange = ManifoldSolid(base_geometry.mold_flange);
        let mut base_sealing_profile_blank = ManifoldSolid(base_geometry.sealing_profile);
        let edge_fastener_holes = build_edge_fastener_holes(
            &spec,
            &ranges,
            FlangeFastenerBand {
                inner_margin: shell_settings.thickness,
                outer_margin: FLANGE_MARGIN,
            },
            PARTING_FLANGE_HALF_DEPTH,
            segment_flanges.top_ramp_length(),
            &generator.flange_fasteners,
        )?;
        validate_cutters_intersect_pair(
            &kernel,
            &lower_flange,
            &upper_flange,
            edge_fastener_holes.iter().map(|hole| &hole.solid),
            "longitudinal parting flange hole",
        )?;
        let base_fastener_band = FlangeFastenerBand {
            inner_margin: shell_settings.thickness,
            outer_margin: segment_flanges.lateral_flange_margin(shell_settings.thickness),
        };
        let base_fastener_holes: Vec<BaseFastenerHole> =
            transverse_through_flange_fastener_cutters(
                &spec,
                root_span,
                base_fastener_band.inner_margin,
                base_fastener_band.outer_margin,
                segment_flanges.axial_thickness,
                segment_flanges.axial_thickness,
                &generator.flange_fasteners,
            )?
            .into_iter()
            .map(|fastener| BaseFastenerHole {
                surface: fastener.surface,
                chord_fraction: fastener.chord_fraction,
                solid: ManifoldSolid(fastener.cutter),
            })
            .collect();
        let mut inserts = build_registration_inserts(
            &spec,
            &ranges,
            &edge_fastener_holes,
            generator.flange_fasteners.head_diameter,
            registration,
        )?;
        let segment_insert_start = inserts.len();
        inserts.extend(build_transverse_join_registration_inserts(
            &spec,
            &ranges,
            &tiles,
            base_fastener_band,
            segment_flanges,
            shell_settings.thickness,
            &generator.flange_fasteners,
            WingTransverseRegistrationSettings::default(),
        )?);
        inserts.extend(build_longitudinal_join_registration_inserts(
            &spec,
            &ranges,
            &tiles,
            base_fastener_band,
            segment_flanges,
            &generator.flange_fasteners,
            WingLongitudinalRegistrationSettings::default(),
        )?);
        let base_insert_start = inserts.len();
        inserts.extend(build_base_registration_inserts(
            &spec,
            root_span,
            &ranges,
            &tiles,
            base_fastener_band,
            &base_fastener_holes,
            generator.flange_fasteners.head_diameter,
            WingTransverseRegistrationSettings::default(),
        )?);
        validate_cutters_intersect_pair(
            &kernel,
            &base_mold_flange,
            &base_sealing_profile_blank,
            inserts[base_insert_start..]
                .iter()
                .map(|insert| &insert.solid),
            "base registration fixture",
        )?;
        validate_disjoint(
            &kernel,
            edge_fastener_holes.iter().map(|hole| &hole.solid),
            inserts[..base_insert_start].iter(),
            "longitudinal fastener hole overlaps a registration fixture",
        )?;
        validate_disjoint(
            &kernel,
            base_fastener_holes.iter().map(|hole| &hole.solid),
            inserts[base_insert_start..].iter(),
            "base fastener hole overlaps a registration fixture",
        )?;
        for cutter in base_fastener_holes.iter().map(|hole| &hole.solid) {
            cut_required(
                &kernel,
                &mut base_mold_flange,
                cutter,
                "base mold flange through hole",
            )?;
            cut_required(
                &kernel,
                &mut base_sealing_profile_blank,
                cutter,
                "base sealing flange through hole",
            )?;
        }
        println!(
            "placed {} registration inserts ({} segment join, {} base)",
            inserts.len(),
            base_insert_start - segment_insert_start,
            inserts.len() - base_insert_start
        );
        let socket_cutters: Vec<&ManifoldSolid> = inserts[..segment_insert_start]
            .iter()
            .map(|insert| &insert.solid)
            .chain(
                inserts[base_insert_start..]
                    .iter()
                    .map(|insert| &insert.solid),
            )
            .chain(edge_fastener_holes.iter().map(|hole| &hole.solid))
            .collect();
        let base_socket_cutters: Vec<&ManifoldSolid> = inserts[base_insert_start..]
            .iter()
            .map(|insert| &insert.solid)
            .collect();
        let mut mold = generate_sectioned_shell_mold_with_parting_ranges(
            &kernel,
            &base_part,
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
        let base_sealing_profile = attach_base_sealing_profile(
            &kernel,
            BaseMoldHalves::first_in(&mut mold).ok_or("mold has no root segments")?,
            BaseAttachmentGeometry {
                opening: &base_opening,
                mold_flange: &base_mold_flange,
                sealing_profile: &base_sealing_profile_blank,
                negative_region: &lower_region,
                positive_region: &upper_region,
                sockets: &base_socket_cutters,
            },
        )?;
        add_segment_join_flanges(
            &kernel,
            &spec,
            &part,
            &lower_region,
            &upper_region,
            &mut mold,
            &ranges,
            &tiles,
            segment_flanges,
            shell_settings.thickness,
            &generator.flange_fasteners,
            &socket_cutters,
        )?;
        if let Some(heads) = &rivet_heads {
            cut_surface_details(&kernel, &mut mold, heads)?;
        }
        clear_part_from_mold(&kernel, &part, &mut mold)?;
        let mut mold = split_mold_into_tiles(&kernel, &spec, mold, &ranges, &tiles)?;
        cut_registration_mating_sockets(
            &kernel,
            &mut mold,
            &inserts[segment_insert_start..base_insert_start],
        )?;
        let fit_failures =
            final_tile_fit_failures(&spec, segmentation.print_volume, &tiles, &mold)?;
        if !fit_failures.is_empty() {
            for failure in fit_failures {
                println!(
                    "rejecting final {} tile {} {:?}: measured dimensions {:?} exceed {:?}",
                    failure.half,
                    failure.tile_index + 1,
                    failure.tile,
                    failure.dimensions,
                    segmentation.print_volume
                );
                if !blocked_tiles.contains(&failure.tile) {
                    blocked_tiles.push(failure.tile);
                }
            }
            continue;
        }
        println!(
            "accepted {} print tiles after measuring both final mold halves",
            tiles.len()
        );
        validate_no_part_intrusion(&part, &mold)?;
        validate_root_alignment(&kernel, &part, &mold, &base_sealing_profile)?;
        mold_wing_geometry::write_stl(
            &part.0,
            output.join(format!("{}.stl", generator.artifact_stem)),
        )?;
        export_artifacts(
            &kernel,
            MoldArtifacts {
                output,
                part: &part,
                mold: &mold,
                base_sealing_profile: &base_sealing_profile,
                inserts: &inserts,
                artifact_stem: &generator.artifact_stem,
                assembly_title: &generator.assembly_title,
            },
        )?;
        return Ok(());
    }
}

fn cut_surface_details(
    kernel: &ManifoldKernel,
    mold: &mut SectionedTwoPartMold<ManifoldSolid>,
    details: &ManifoldSolid,
) -> Result<(), mold_manifold::ManifoldKernelError> {
    for piece in mold.negative.iter_mut().chain(&mut mold.positive) {
        *piece = kernel.difference(piece, details)?;
    }
    Ok(())
}

fn clear_part_from_mold(
    kernel: &ManifoldKernel,
    part: &ManifoldSolid,
    mold: &mut SectionedTwoPartMold<ManifoldSolid>,
) -> Result<(), mold_manifold::ManifoldKernelError> {
    for piece in mold.negative.iter_mut().chain(&mut mold.positive) {
        *piece = kernel.difference(piece, part)?;
    }
    Ok(())
}

fn validate_root_alignment(
    kernel: &ManifoldKernel,
    part: &ManifoldSolid,
    mold: &SectionedTwoPartMold<ManifoldSolid>,
    sealing_profile: &ManifoldSolid,
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
    let sealing_face = kernel.bounds(sealing_profile)?.max.y;
    if (mold_root - wing_root).abs() > 1.0e-6 || (sealing_face - wing_root).abs() > 1.0e-6 {
        return Err(format!(
            "wing, mold, and sealing profile do not share the root plane: wing={wing_root:.6}, mold={mold_root:.6}, seal={sealing_face:.6}"
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
    fasteners: &WingFlangeFastenerSpec,
    exclusions: &[&ManifoldSolid],
) -> Result<(), Box<dyn std::error::Error>> {
    if !shell_thickness.is_finite()
        || shell_thickness <= 0.0
        || !settings.width.is_finite()
        || settings.width <= shell_thickness
        || settings.axial_thickness <= 0.0
        || !(0.0..90.0).contains(&settings.maximum_overhang_angle_deg)
    {
        return Err("invalid segment flange settings".into());
    }
    let model_start = spec.stations.first().ok_or("wing has no stations")?.span;
    let model_end = spec.stations.last().ok_or("wing has no stations")?.span;
    let longitudinal_attachment_offset =
        SegmentFlangeSettings::longitudinal_attachment_offset(shell_thickness);
    let lateral_flange_margin = settings.lateral_flange_margin(shell_thickness);
    let longitudinal_fastener_band = FlangeFastenerBand {
        inner_margin: shell_thickness,
        outer_margin: lateral_flange_margin,
    };
    let longitudinal_flange_thickness = settings.longitudinal_flange_thickness();

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
            &[
                (span - ramp, shell_thickness),
                (span, lateral_flange_margin),
            ],
        )?);
        let bed_blank = ManifoldSolid(transverse_flange_blank(
            spec,
            &[
                (span, lateral_flange_margin),
                (span + settings.axial_thickness, lateral_flange_margin),
            ],
        )?);
        for (pieces, region) in [
            (&mut mold.negative, lower_region),
            (&mut mold.positive, upper_region),
        ] {
            attach(&mut pieces[seam], top_blank.clone(), region)?;
            attach(&mut pieces[seam + 1], bed_blank.clone(), region)?;
        }
        let fastener_cutters = transverse_flange_fastener_cutters(
            spec,
            span,
            shell_thickness,
            lateral_flange_margin,
            ramp,
            settings.axial_thickness,
            fasteners,
        )?;
        for cutters in fastener_cutters {
            let pieces = match cutters.surface {
                WingSurface::Lower => &mut mold.negative,
                WingSurface::Upper => &mut mold.positive,
            };
            cut_required(
                kernel,
                &mut pieces[seam],
                &ManifoldSolid(cutters.pilot),
                "blind pilot",
            )?;
            cut_required(
                kernel,
                &mut pieces[seam + 1],
                &ManifoldSolid(cutters.clearance),
                "bed clearance",
            )?;
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
            let end_obstructions = longitudinal_flange_end_obstructions(
                section + 1 < ranges.len(),
                settings.top_ramp_length(),
            );
            let upper_direction =
                segment_normal(spec, range.0.max(model_start), range.1.min(model_end))?;
            let lower_direction = upper_direction.map(|value| -value);
            let start_normal = transverse_section_normal(spec, range.0)?;
            let end_normal = transverse_section_normal(spec, range.1)?;
            let make_flange =
                |surface, direction| -> Result<ManifoldSolid, Box<dyn std::error::Error>> {
                    let path = sample_longitudinal_surface_path(
                        spec,
                        chord_fraction,
                        range.0,
                        range.1,
                        SURFACE_SAMPLES,
                        surface,
                    )?;
                    let start_plane = SweepEndPlane {
                        point: path[0],
                        normal: start_normal,
                    };
                    let end_plane = SweepEndPlane {
                        point: path[path.len() - 1],
                        normal: end_normal,
                    };
                    let path: Vec<[f64; 3]> = path
                        .into_iter()
                        .map(|point| add_scaled(point, direction, longitudinal_attachment_offset))
                        .collect();
                    Ok(kernel.swept_rib_with_end_planes(
                        &path,
                        direction,
                        longitudinal_flange_thickness,
                        settings.width,
                        start_plane,
                        end_plane,
                    )?)
                };
            let lower = make_flange(WingSurface::Lower, lower_direction)?;
            let upper = make_flange(WingSurface::Upper, upper_direction)?;
            mold.negative[section] = kernel.union_attached(&mold.negative[section], &lower)?;
            mold.positive[section] = kernel.union_attached(&mold.positive[section], &upper)?;
            let lower_fasteners = longitudinal_split_flange_fastener_cutters(
                spec,
                chord_fraction,
                *range,
                WingSurface::Lower,
                lower_direction,
                longitudinal_fastener_band,
                longitudinal_flange_thickness,
                end_obstructions,
                fasteners,
            )?;
            let upper_fasteners = longitudinal_split_flange_fastener_cutters(
                spec,
                chord_fraction,
                *range,
                WingSurface::Upper,
                upper_direction,
                longitudinal_fastener_band,
                longitudinal_flange_thickness,
                end_obstructions,
                fasteners,
            )?;
            for fastener in lower_fasteners {
                cut_required(
                    kernel,
                    &mut mold.negative[section],
                    &ManifoldSolid(fastener.cutter),
                    "lower longitudinal split flange through hole",
                )?;
            }
            for fastener in upper_fasteners {
                cut_required(
                    kernel,
                    &mut mold.positive[section],
                    &ManifoldSolid(fastener.cutter),
                    "upper longitudinal split flange through hole",
                )?;
            }
        }
    }
    for piece in mold.negative.iter_mut().chain(&mut mold.positive) {
        for exclusion in exclusions {
            *piece = kernel.difference(piece, exclusion)?;
        }
    }
    Ok(())
}

fn cut_required(
    kernel: &ManifoldKernel,
    piece: &mut ManifoldSolid,
    cutter: &ManifoldSolid,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    if kernel.intersection(piece, cutter)?.0.volume() <= 1.0e-9 {
        return Err(format!("{label} cutter does not intersect its flange").into());
    }
    *piece = kernel.difference(piece, cutter)?;
    Ok(())
}

fn cut_registration_mating_sockets(
    kernel: &ManifoldKernel,
    mold: &mut SectionedTwoPartMold<ManifoldSolid>,
    inserts: &[RegistrationInsert],
) -> Result<(), Box<dyn std::error::Error>> {
    for insert in inserts {
        let mating = insert
            .mating
            .ok_or("segment registration fixture is missing its mating tiles")?;
        if mating.tile_indices[0] == mating.tile_indices[1] {
            return Err("segment registration fixture targets the same tile twice".into());
        }
        let pieces = match mating.surface {
            WingSurface::Lower => &mut mold.negative,
            WingSurface::Upper => &mut mold.positive,
        };
        let label = format!("{} socket", insert.name);
        for tile_index in mating.tile_indices {
            let piece = pieces
                .get_mut(tile_index)
                .ok_or("segment registration fixture targets a missing tile")?;
            cut_required(kernel, piece, &insert.solid, &label)?;
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
                    cutters.push(ManifoldSolid(sampled_chord_band_region(
                        spec,
                        (0.0, tile.chord.1),
                        *range,
                        -1_000.0,
                        1_000.0,
                        100.0,
                        SURFACE_SAMPLES,
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

fn build_edge_fastener_holes(
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    band: FlangeFastenerBand,
    half_depth: f64,
    sloped_end_length: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<EdgeFastenerHole>, Box<dyn std::error::Error>> {
    let mut holes = Vec::new();
    for (section, &range) in ranges.iter().enumerate() {
        let end_obstructions =
            longitudinal_flange_end_obstructions(section + 1 < ranges.len(), sloped_end_length);
        for edge in [FlangeEdge::Leading, FlangeEdge::Trailing] {
            holes.extend(
                longitudinal_edge_fastener_cutters(
                    spec,
                    wing_edge(edge),
                    range,
                    band,
                    half_depth,
                    end_obstructions,
                    fasteners,
                )?
                .into_iter()
                .map(|fastener| EdgeFastenerHole {
                    section,
                    edge,
                    position: fastener.position,
                    solid: ManifoldSolid(fastener.cutter),
                }),
            );
        }
    }
    Ok(holes)
}

/// Internal sections end against the sloped lateral flange. Their following
/// sections start against the bed-oriented flange, so only the far end needs
/// additional fastener-head clearance.
fn longitudinal_flange_end_obstructions(
    has_following_section: bool,
    sloped_end_length: f64,
) -> FlangeEndObstructions {
    FlangeEndObstructions {
        end: if has_following_section {
            sloped_end_length
        } else {
            0.0
        },
        ..Default::default()
    }
}

fn build_registration_inserts(
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    fastener_holes: &[EdgeFastenerHole],
    fastener_head_diameter: f64,
    settings: RegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    let mut inserts = Vec::new();
    for (segment, _) in ranges.iter().enumerate() {
        for edge in [FlangeEdge::Leading, FlangeEdge::Trailing] {
            let fixture = match edge {
                FlangeEdge::Leading => settings.leading,
                FlangeEdge::Trailing => settings.trailing,
            };
            let positions: Vec<f64> = fastener_holes
                .iter()
                .filter(|hole| hole.section == segment && hole.edge == edge)
                .map(|hole| hole.position)
                .collect();
            for (index, pair) in positions.windows(2).enumerate() {
                let span = (pair[0] + pair[1]) * 0.5;
                let available_half_span = (pair[1] - pair[0]) * 0.5;
                if available_half_span <= fixture.span_half_width + fastener_head_diameter * 0.5 {
                    continue;
                }
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
                    mating: None,
                });
            }
        }
    }
    Ok(inserts)
}

#[allow(clippy::too_many_arguments)]
fn build_transverse_join_registration_inserts(
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
    band: FlangeFastenerBand,
    flange_settings: SegmentFlangeSettings,
    shell_thickness: f64,
    fasteners: &WingFlangeFastenerSpec,
    settings: WingTransverseRegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    let mut inserts = Vec::new();
    for seam in 0..ranges.len().saturating_sub(1) {
        let span = ranges[seam].1;
        let fastener_cutters = transverse_flange_fastener_cutters(
            spec,
            span,
            shell_thickness,
            band.outer_margin,
            flange_settings.top_ramp_length(),
            flange_settings.axial_thickness,
            fasteners,
        )?;
        let chord_ranges = transverse_join_chord_ranges(ranges, tiles, seam)?;
        for (chord_index, chord_range) in chord_ranges.into_iter().enumerate() {
            let chord_midpoint = (chord_range.0 + chord_range.1) * 0.5;
            let tile_indices = [
                tile_index_at(tiles, ranges[seam], chord_midpoint)?,
                tile_index_at(tiles, ranges[seam + 1], chord_midpoint)?,
            ];
            for surface in [WingSurface::Lower, WingSurface::Upper] {
                let fastener_positions: Vec<f64> = fastener_cutters
                    .iter()
                    .filter(|cutter| cutter.surface == surface)
                    .map(|cutter| cutter.chord_fraction)
                    .collect();
                let fixtures = transverse_flange_registration_inserts(
                    spec,
                    span,
                    surface,
                    chord_range,
                    band,
                    &fastener_positions,
                    fasteners.head_diameter,
                    settings,
                )?;
                let side = wing_surface_name(surface);
                inserts.extend(
                    fixtures
                        .into_iter()
                        .enumerate()
                        .map(|(fixture_index, solid)| RegistrationInsert {
                            name: format!(
                                "registration-insert-transverse-s{:02}-{side}-c{:02}-{:02}",
                                seam + 1,
                                chord_index + 1,
                                fixture_index + 1
                            ),
                            solid: ManifoldSolid(solid),
                            mating: Some(RegistrationMating {
                                surface,
                                tile_indices,
                            }),
                        }),
                );
            }
        }
    }
    Ok(inserts)
}

fn tile_index_at(
    tiles: &[PrintTile],
    span: (f64, f64),
    chord_fraction: f64,
) -> Result<usize, Box<dyn std::error::Error>> {
    tiles
        .iter()
        .position(|tile| {
            tile.span == span
                && chord_fraction > tile.chord.0 - 1.0e-9
                && chord_fraction < tile.chord.1 + 1.0e-9
        })
        .ok_or_else(|| "registration position does not belong to a print tile".into())
}

fn transverse_join_chord_ranges(
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
    seam: usize,
) -> Result<Vec<(f64, f64)>, Box<dyn std::error::Error>> {
    let adjoining = ranges
        .get(seam..=seam + 1)
        .ok_or("transverse join has no adjoining span ranges")?;
    let mut boundaries = vec![0.0, 1.0];
    for range in adjoining {
        let section_tiles: Vec<&PrintTile> =
            tiles.iter().filter(|tile| tile.span == *range).collect();
        if section_tiles.is_empty() {
            return Err("transverse join has an adjoining span range without tiles".into());
        }
        for tile in section_tiles {
            boundaries.extend([tile.chord.0, tile.chord.1]);
        }
    }
    boundaries.sort_by(f64::total_cmp);
    boundaries.dedup_by(|a, b| (*a - *b).abs() < 1.0e-9);
    Ok(boundaries
        .windows(2)
        .filter(|pair| pair[1] - pair[0] > 1.0e-9)
        .map(|pair| (pair[0], pair[1]))
        .collect())
}

#[allow(clippy::too_many_arguments)]
fn build_longitudinal_join_registration_inserts(
    spec: &WingSpec,
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
    band: FlangeFastenerBand,
    flange_settings: SegmentFlangeSettings,
    fasteners: &WingFlangeFastenerSpec,
    settings: WingLongitudinalRegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    if settings.seam_half_depth > flange_settings.axial_thickness {
        return Err("longitudinal registration depth exceeds each tile's flange thickness".into());
    }
    let model_start = spec.stations.first().ok_or("wing has no stations")?.span;
    let model_end = spec.stations.last().ok_or("wing has no stations")?.span;
    let longitudinal_flange_thickness = flange_settings.longitudinal_flange_thickness();
    let mut inserts = Vec::new();
    for (section, range) in ranges.iter().enumerate() {
        let section_tiles: Vec<&PrintTile> =
            tiles.iter().filter(|tile| tile.span == *range).collect();
        for (chord_index, pair) in section_tiles.windows(2).enumerate() {
            let tile_indices = [
                tiles
                    .iter()
                    .position(|tile| std::ptr::eq(tile, pair[0]))
                    .ok_or("longitudinal registration is missing its leading tile")?,
                tiles
                    .iter()
                    .position(|tile| std::ptr::eq(tile, pair[1]))
                    .ok_or("longitudinal registration is missing its trailing tile")?,
            ];
            let chord_fraction = pair[0].chord.1;
            let end_obstructions = longitudinal_flange_end_obstructions(
                section + 1 < ranges.len(),
                flange_settings.top_ramp_length(),
            );
            let upper_direction =
                segment_normal(spec, range.0.max(model_start), range.1.min(model_end))?;
            for (surface, direction) in [
                (WingSurface::Lower, upper_direction.map(|value| -value)),
                (WingSurface::Upper, upper_direction),
            ] {
                let cutters = longitudinal_split_flange_fastener_cutters(
                    spec,
                    chord_fraction,
                    *range,
                    surface,
                    direction,
                    band,
                    longitudinal_flange_thickness,
                    end_obstructions,
                    fasteners,
                )?;
                let positions: Vec<f64> = cutters.iter().map(|cutter| cutter.position).collect();
                let fixtures = longitudinal_split_flange_registration_inserts(
                    spec,
                    chord_fraction,
                    *range,
                    surface,
                    direction,
                    band,
                    &positions,
                    fasteners.head_diameter,
                    settings,
                )?;
                let side = wing_surface_name(surface);
                inserts.extend(
                    fixtures
                        .into_iter()
                        .enumerate()
                        .map(|(fixture_index, solid)| RegistrationInsert {
                            name: format!(
                                "registration-insert-longitudinal-s{:02}-{side}-c{:02}-{:02}",
                                section + 1,
                                chord_index + 1,
                                fixture_index + 1
                            ),
                            solid: ManifoldSolid(solid),
                            mating: Some(RegistrationMating {
                                surface,
                                tile_indices,
                            }),
                        }),
                );
            }
        }
    }
    Ok(inserts)
}

#[allow(clippy::too_many_arguments)]
fn build_base_registration_inserts(
    spec: &WingSpec,
    root_span: f64,
    ranges: &[(f64, f64)],
    tiles: &[PrintTile],
    band: FlangeFastenerBand,
    fastener_holes: &[BaseFastenerHole],
    fastener_head_diameter: f64,
    settings: WingTransverseRegistrationSettings,
) -> Result<Vec<RegistrationInsert>, Box<dyn std::error::Error>> {
    let root_range = ranges.first().ok_or("mold has no root segment")?;
    let root_tiles: Vec<&PrintTile> = tiles
        .iter()
        .filter(|tile| tile.span == *root_range)
        .collect();
    if root_tiles.is_empty() {
        return Err("mold has no root tiles".into());
    }

    let mut inserts = Vec::with_capacity(root_tiles.len() * 2 * settings.fixtures_per_flange);
    for (tile_index, tile) in root_tiles.into_iter().enumerate() {
        for surface in [WingSurface::Lower, WingSurface::Upper] {
            let fastener_positions: Vec<f64> = fastener_holes
                .iter()
                .filter(|hole| hole.surface == surface)
                .map(|hole| hole.chord_fraction)
                .collect();
            let fixtures = transverse_flange_registration_inserts(
                spec,
                root_span,
                surface,
                tile.chord,
                band,
                &fastener_positions,
                fastener_head_diameter,
                settings,
            )?;
            let side = wing_surface_name(surface);
            inserts.extend(
                fixtures
                    .into_iter()
                    .enumerate()
                    .map(|(fixture_index, solid)| RegistrationInsert {
                        name: format!(
                            "registration-insert-base-{side}-c{:02}-{:02}",
                            tile_index + 1,
                            fixture_index + 1
                        ),
                        solid: ManifoldSolid(solid),
                        mating: None,
                    }),
            );
        }
    }
    Ok(inserts)
}

fn validate_cutters_intersect_pair<'a, I>(
    kernel: &ManifoldKernel,
    first: &ManifoldSolid,
    second: &ManifoldSolid,
    cutters: I,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>>
where
    I: IntoIterator<Item = &'a ManifoldSolid>,
{
    for cutter in cutters {
        if kernel.intersection(first, cutter)?.0.volume() <= 1.0e-9
            || kernel.intersection(second, cutter)?.0.volume() <= 1.0e-9
        {
            return Err(format!("{label} does not intersect both mating flanges").into());
        }
    }
    Ok(())
}

fn validate_disjoint<'a, 'b, C, F>(
    kernel: &ManifoldKernel,
    cutters: C,
    fixtures: F,
    label: &str,
) -> Result<(), Box<dyn std::error::Error>>
where
    C: IntoIterator<Item = &'a ManifoldSolid>,
    F: IntoIterator<Item = &'b RegistrationInsert>,
{
    let cutters: Vec<&ManifoldSolid> = cutters.into_iter().collect();
    for fixture in fixtures {
        for cutter in &cutters {
            if kernel.intersection(cutter, &fixture.solid)?.0.volume() > 1.0e-9 {
                return Err(label.into());
            }
        }
    }
    Ok(())
}

fn export_artifacts(
    kernel: &ManifoldKernel,
    artifacts: MoldArtifacts<'_>,
) -> Result<(), Box<dyn std::error::Error>> {
    let MoldArtifacts {
        output,
        part,
        mold,
        base_sealing_profile,
        inserts,
        artifact_stem,
        assembly_title,
    } = artifacts;
    for (prefix, pieces) in [("lower", &mold.negative), ("upper", &mold.positive)] {
        for (index, piece) in pieces.iter().enumerate() {
            kernel.export_stl(
                piece,
                output.join(format!("mold-{prefix}-{:02}.stl", index + 1)),
            )?;
        }
    }
    kernel.export_stl(
        base_sealing_profile,
        output.join("base-sealing-profile.stl"),
    )?;
    for insert in inserts {
        kernel.export_stl(&insert.solid, output.join(format!("{}.stl", insert.name)))?;
    }
    let mut assembly = vec![
        ThreeMfObject {
            name: "wing".to_owned(),
            solid: part,
        },
        ThreeMfObject {
            name: "base-sealing-profile".to_owned(),
            solid: base_sealing_profile,
        },
    ];
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
        output.join(format!("{artifact_stem}-mold-assembly.3mf")),
        assembly_title,
        &assembly,
    )?;
    println!("generated wing mold in {}", output.display());
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

const fn wing_surface_name(surface: WingSurface) -> &'static str {
    match surface {
        WingSurface::Lower => "lower",
        WingSurface::Upper => "upper",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mold_geometry::{Bounds3, Vec3};

    #[test]
    fn wing_shell_is_four_millimetres_without_webbing() {
        let settings = wing_shell_settings(0.0);

        assert_eq!(settings.thickness, 4.0);
        assert!(settings.structural_webbing.is_none());
    }

    #[test]
    fn final_cavity_recut_removes_part_intrusion_from_every_half() {
        let kernel = ManifoldKernel;
        let cuboid = |minimum, maximum| {
            kernel
                .cuboid(Bounds3 {
                    min: Vec3::new(minimum, minimum, minimum),
                    max: Vec3::new(maximum, maximum, maximum),
                })
                .unwrap()
        };
        let part = cuboid(0.0, 10.0);
        let outer = cuboid(-1.0, 11.0);
        let mut mold = SectionedTwoPartMold {
            negative: vec![outer.clone()],
            positive: vec![outer],
        };

        clear_part_from_mold(&kernel, &part, &mut mold).unwrap();

        for piece in mold.negative.iter().chain(&mold.positive) {
            assert!(kernel.intersection(piece, &part).unwrap().0.volume() <= 1.0e-9);
        }
    }

    #[test]
    fn final_fit_check_rejects_actual_meshes_that_exceed_the_estimate() {
        let kernel = ManifoldKernel;
        let oversized = kernel
            .cuboid(Bounds3 {
                min: Vec3::new(0.0, 0.0, 0.0),
                max: Vec3::new(400.0, 100.0, 20.0),
            })
            .unwrap();
        let tile = PrintTile {
            span: (0.0, 100.0),
            chord: (0.0, 1.0),
        };
        let mold = SectionedTwoPartMold {
            negative: vec![oversized.clone()],
            positive: vec![oversized],
        };
        let failures = final_tile_fit_failures(
            &mold_wing_geometry::preset("rectangular").unwrap(),
            PrintVolume {
                width: 256.0,
                depth: 256.0,
                height: 256.0,
                clearance: 6.0,
            },
            &[tile],
            &mold,
        )
        .unwrap();

        assert_eq!(failures.len(), 2);
        assert!(failures.iter().all(|failure| failure.tile == tile));
        assert!(failures.iter().all(|failure| failure.dimensions[0] > 244.0));
    }

    #[test]
    fn rejected_estimated_tile_is_excluded_from_the_next_plan() {
        let spec = mold_wing_geometry::preset("rectangular").unwrap();
        let boundaries: Vec<SegmentBoundary> = (0..=6)
            .map(|index| SegmentBoundary {
                position: index as f64 * 100.0,
                preference: 0.0,
            })
            .collect();
        let settings = TiledSegmentationSettings {
            span: SegmentationSettings {
                print_volume: PrintVolume {
                    width: 256.0,
                    depth: 256.0,
                    height: 256.0,
                    clearance: 6.0,
                },
                preferred_segment_count: None,
                max_segment_count: 8,
            },
            max_longitudinal_segments: 4,
        };
        let envelope = PrintableEnvelope {
            flange_margin: FLANGE_MARGIN,
            shell_thickness: 4.0,
            web_depth: SegmentFlangeSettings::default().width,
            span_samples: SURFACE_SAMPLES,
        };
        let flange_settings = SegmentFlangeSettings::default();
        let fasteners = WingFlangeFastenerSpec::default();
        let initial = plan_print_tiles(
            &spec,
            &boundaries,
            settings,
            envelope,
            flange_settings,
            &fasteners,
            &[],
        )
        .unwrap();
        let rejected = initial[0];

        let replanned = plan_print_tiles(
            &spec,
            &boundaries,
            settings,
            envelope,
            flange_settings,
            &fasteners,
            &[rejected],
        )
        .unwrap();

        assert!(!replanned.contains(&rejected));
        assert_ne!(replanned, initial);
    }

    #[test]
    fn scaled_gull_sample_preserves_the_bend_and_keeps_the_narrow_tip_whole() {
        let mut spec = mold_wing_geometry::preset("gull").unwrap();
        for station in &mut spec.stations {
            station.span *= 1.37;
            station.chord *= 1.37;
            station.x_offset *= 1.37;
            station.z_offset *= 1.37;
        }
        spec.profile_points = 24;
        let shell_settings = wing_shell_settings(0.0);
        let boundaries: Vec<SegmentBoundary> = wing_segment_boundaries(
            &spec,
            spec.stations.first().unwrap().span,
            spec.stations.last().unwrap().span + shell_settings.thickness,
            CANDIDATE_STEP,
        )
        .unwrap()
        .into_iter()
        .map(planning_boundary)
        .collect();
        let tiles = plan_print_tiles(
            &spec,
            &boundaries,
            TiledSegmentationSettings {
                span: SegmentationSettings {
                    print_volume: PrintVolume {
                        width: 256.0,
                        depth: 256.0,
                        height: 256.0,
                        clearance: 6.0,
                    },
                    preferred_segment_count: None,
                    max_segment_count: 8,
                },
                max_longitudinal_segments: 4,
            },
            PrintableEnvelope {
                flange_margin: FLANGE_MARGIN,
                shell_thickness: shell_settings.thickness,
                web_depth: SegmentFlangeSettings::default().width,
                span_samples: SURFACE_SAMPLES,
            },
            SegmentFlangeSettings::default(),
            &WingFlangeFastenerSpec::default(),
            &[],
        )
        .unwrap();

        assert!(tiles.len() > 5, "unexpected five-tile plan: {tiles:?}");
        let bend_span = spec.stations[1].span;
        assert!(
            tiles
                .iter()
                .any(|tile| (tile.span.1 - bend_span).abs() < 1.0e-9),
            "plan does not end a section at the gull bend: {tiles:?}"
        );
        let tip_range = tiles.last().unwrap().span;
        let tip_tiles: Vec<&PrintTile> =
            tiles.iter().filter(|tile| tile.span == tip_range).collect();
        assert_eq!(tip_tiles.len(), 1, "narrow tip was split: {tiles:?}");
        assert_eq!(tip_tiles[0].chord, (0.0, 1.0));
    }

    #[test]
    fn spitfire_plan_absorbs_a_tip_too_short_for_flange_fasteners() {
        let spec = mold_wing_geometry::preset("elliptical").unwrap();
        let shell_settings = wing_shell_settings(0.35);
        let flange_settings = SegmentFlangeSettings::default();
        let fasteners = WingFlangeFastenerSpec::default();
        let final_span = spec.stations.last().unwrap().span + shell_settings.thickness;
        let print_volume = PrintVolume {
            width: 256.0,
            depth: 256.0,
            height: 256.0,
            clearance: 6.0,
        };
        let envelope = PrintableEnvelope {
            flange_margin: FLANGE_MARGIN,
            shell_thickness: shell_settings.thickness,
            web_depth: flange_settings.width,
            span_samples: SURFACE_SAMPLES,
        };
        let boundaries: Vec<SegmentBoundary> = wing_segment_boundaries(
            &spec,
            spec.stations.first().unwrap().span,
            final_span,
            CANDIDATE_STEP,
        )
        .unwrap()
        .into_iter()
        .map(planning_boundary)
        .collect();
        let tiles = plan_print_tiles(
            &spec,
            &boundaries,
            TiledSegmentationSettings {
                span: SegmentationSettings {
                    print_volume,
                    preferred_segment_count: None,
                    max_segment_count: 8,
                },
                max_longitudinal_segments: 4,
            },
            envelope,
            flange_settings,
            &fasteners,
            &[],
        )
        .unwrap();
        let mut ranges = Vec::new();
        for tile in &tiles {
            if ranges.last() != Some(&tile.span) {
                ranges.push(tile.span);
            }
        }

        assert_eq!(
            ranges.len(),
            3,
            "Spitfire plan is over-segmented: {ranges:?}"
        );
        let usable_height = print_volume.height - 2.0 * print_volume.clearance;
        for tile in &tiles {
            let dimensions =
                printable_tile_dimensions(&spec, tile.span.0, tile.span.1, tile.chord, envelope)
                    .unwrap();
            assert!(
                dimensions[2] >= usable_height * 0.75,
                "Spitfire tile underuses print-oriented Z: {dimensions:?}"
            );
        }
        assert!(
            !ranges.iter().any(|range| {
                (range.0 - 582.0).abs() < 1.0e-9 && (range.1 - final_span).abs() < 1.0e-9
            }),
            "undersized Spitfire tip survived planning: {ranges:?}"
        );
        for (index, range) in ranges.iter().copied().enumerate() {
            let obstructions = longitudinal_flange_end_obstructions(
                index + 1 < ranges.len(),
                flange_settings.top_ramp_length(),
            );
            assert!(
                flange_fastener_positions(range, obstructions, &fasteners).is_ok(),
                "planned flange cannot carry its fasteners: {range:?}"
            );
        }
    }

    #[test]
    fn wing_shell_adds_surface_detail_height_to_preserve_minimum_wall() {
        let settings = wing_shell_settings(0.35);

        assert_eq!(settings.thickness, 4.35);
    }

    #[test]
    fn only_internal_section_ends_are_obstructed_by_a_sloped_flange() {
        assert_eq!(
            longitudinal_flange_end_obstructions(true, 12.0),
            FlangeEndObstructions {
                start: 0.0,
                end: 12.0,
            }
        );
        assert_eq!(
            longitudinal_flange_end_obstructions(false, 12.0),
            FlangeEndObstructions::default()
        );
    }

    #[test]
    fn longitudinal_registration_stays_within_each_tile_flange() {
        let flange = SegmentFlangeSettings::default();
        let registration = WingLongitudinalRegistrationSettings::default();

        assert_eq!(
            flange.longitudinal_flange_thickness() * 0.5,
            flange.axial_thickness
        );
        assert!(registration.seam_half_depth < flange.axial_thickness);
    }

    #[test]
    fn gull_root_gets_two_registration_fixtures_per_constituent_flange() {
        let spec = mold_wing_geometry::preset("gull").unwrap();
        let fasteners = WingFlangeFastenerSpec::default();
        let band = FlangeFastenerBand {
            inner_margin: 4.0,
            outer_margin: 14.4,
        };
        let fastener_holes: Vec<BaseFastenerHole> = transverse_through_flange_fastener_cutters(
            &spec,
            0.0,
            band.inner_margin,
            band.outer_margin,
            3.0,
            3.0,
            &fasteners,
        )
        .unwrap()
        .into_iter()
        .map(|fastener| BaseFastenerHole {
            surface: fastener.surface,
            chord_fraction: fastener.chord_fraction,
            solid: ManifoldSolid(fastener.cutter),
        })
        .collect();
        let ranges = [(0.0, 216.0), (216.0, 400.0)];
        let tiles = [
            PrintTile {
                span: ranges[0],
                chord: (0.0, 0.5),
            },
            PrintTile {
                span: ranges[0],
                chord: (0.5, 1.0),
            },
            PrintTile {
                span: ranges[1],
                chord: (0.0, 1.0),
            },
        ];

        let inserts = build_base_registration_inserts(
            &spec,
            0.0,
            &ranges,
            &tiles,
            band,
            &fastener_holes,
            fasteners.head_diameter,
            WingTransverseRegistrationSettings::default(),
        )
        .unwrap();

        assert_eq!(inserts.len(), 8);
        assert_eq!(
            inserts
                .iter()
                .filter(|insert| insert.name.contains("-lower-"))
                .count(),
            4
        );
        assert_eq!(
            inserts
                .iter()
                .filter(|insert| insert.name.contains("-upper-"))
                .count(),
            4
        );
    }

    #[test]
    fn gull_segment_joins_get_registration_on_every_mating_flange() {
        let spec = mold_wing_geometry::preset("gull").unwrap();
        let ranges = [
            (0.0, 123.14329738058551),
            (123.14329738058551, 324.55172413793105),
            (324.55172413793105, 524.2758620689655),
            (524.2758620689655, 724.0),
        ];
        let tiles = [
            PrintTile {
                span: ranges[0],
                chord: (0.0, 0.5),
            },
            PrintTile {
                span: ranges[0],
                chord: (0.5, 1.0),
            },
            PrintTile {
                span: ranges[1],
                chord: (0.0, 1.0),
            },
            PrintTile {
                span: ranges[2],
                chord: (0.0, 1.0),
            },
            PrintTile {
                span: ranges[3],
                chord: (0.0, 1.0),
            },
        ];
        let band = FlangeFastenerBand {
            inner_margin: 4.0,
            outer_margin: 15.2,
        };
        let flange_settings = SegmentFlangeSettings::default();
        let fasteners = WingFlangeFastenerSpec::default();

        let transverse = build_transverse_join_registration_inserts(
            &spec,
            &ranges,
            &tiles,
            band,
            flange_settings,
            4.0,
            &fasteners,
            WingTransverseRegistrationSettings::default(),
        )
        .unwrap();
        let longitudinal = build_longitudinal_join_registration_inserts(
            &spec,
            &ranges,
            &tiles,
            band,
            flange_settings,
            &fasteners,
            WingLongitudinalRegistrationSettings::default(),
        )
        .unwrap();

        assert_eq!(transverse.len(), 16);
        assert_eq!(longitudinal.len(), 2);
        for insert in transverse.iter().chain(&longitudinal) {
            let mating = insert.mating.expect("segment insert needs mating tiles");
            assert_ne!(mating.tile_indices[0], mating.tile_indices[1]);
        }
    }

    #[test]
    fn root_alignment_rejects_an_offset_sealing_face() {
        let kernel = ManifoldKernel;
        let cuboid = |min_y, max_y| {
            kernel
                .cuboid(Bounds3 {
                    min: Vec3::new(0.0, min_y, 0.0),
                    max: Vec3::new(10.0, max_y, 10.0),
                })
                .unwrap()
        };
        let part = cuboid(0.0, 10.0);
        let mold = SectionedTwoPartMold {
            negative: vec![cuboid(0.0, 10.0)],
            positive: vec![cuboid(0.0, 10.0)],
        };
        let aligned_seal = cuboid(-3.0, 0.0);
        let offset_seal = cuboid(-4.0, -1.0);

        assert!(validate_root_alignment(&kernel, &part, &mold, &aligned_seal).is_ok());
        assert!(validate_root_alignment(&kernel, &part, &mold, &offset_seal).is_err());
    }
}
