use std::{fs, path::Path};

use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::SolidKernel;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{
    FlangeDivisionSettings, FlangeEdge, PartingRegions, ShellSettings, WebbingSettings,
    alternating_rib_layouts, attach_structural_webbing, divide_flange,
    generate_sectioned_shell_mold_with_parting_ranges,
};
use mold_test_models::{
    RibPathSpec, WingEdge, WingSpec, WingSurface, chord_region, deviation_aware_segment_ranges,
    registration_diamond, sample_rib_surface_path, segment_normal,
};

const SEGMENT_COUNT: usize = 2;
const FLANGE_MARGIN: f64 = 12.0;
const RIB_SAMPLES: usize = 24;

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
    let kernel = ManifoldKernel;
    let part = ManifoldSolid(mold_test_models::generate(&spec)?);
    mold_test_models::write_stl(&part.0, output.join("gull-wing.stl"))?;

    let lower_region = ManifoldSolid(chord_region(&spec, -500.0, 0.0, 80.0)?);
    let upper_region = ManifoldSolid(chord_region(&spec, 0.0, 500.0, 80.0)?);
    let lower_flange = ManifoldSolid(chord_region(&spec, -3.0, 0.0, FLANGE_MARGIN)?);
    let upper_flange = ManifoldSolid(chord_region(&spec, 0.0, 3.0, FLANGE_MARGIN)?);

    let shell_settings = ShellSettings {
        structural_webbing: None,
        ..Default::default()
    };
    let expanded_bounds = kernel.bounds(&kernel.offset(&part, shell_settings.thickness)?)?;
    let ranges = deviation_aware_segment_ranges(
        &spec,
        SEGMENT_COUNT,
        expanded_bounds.min.y,
        expanded_bounds.max.y,
    )?;
    println!("deviation-aware segment ranges: {ranges:?}");

    let registration = RegistrationSettings::default();
    let inserts = build_registration_inserts(&spec, &ranges, registration)?;
    let socket_cutters: Vec<&ManifoldSolid> = inserts.iter().map(|insert| &insert.solid).collect();
    let webbing = WebbingSettings::default();
    let (lower_ribs, upper_ribs) = build_ribs(&kernel, &spec, &ranges, registration, webbing)?;

    let baseline = generate_sectioned_shell_mold_with_parting_ranges(
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

    let lower_webbing = additions(&kernel, &mold.negative, &baseline.negative)?;
    let upper_webbing = additions(&kernel, &mold.positive, &baseline.positive)?;
    validate_attached("lower", &mold.negative, &baseline.negative, &lower_webbing)?;
    validate_attached("upper", &mold.positive, &baseline.positive, &upper_webbing)?;
    export_artifacts(
        &kernel,
        output,
        &part,
        &mold,
        &inserts,
        &lower_webbing,
        &upper_webbing,
    )?;
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
    lower_webbing: &[ManifoldSolid],
    upper_webbing: &[ManifoldSolid],
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
    for (prefix, pieces) in [("lower", lower_webbing), ("upper", upper_webbing)] {
        for (index, piece) in pieces.iter().enumerate() {
            kernel.export_stl(
                piece,
                output.join(format!("webbing-{prefix}-{:02}.stl", index + 1)),
            )?;
        }
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
    for (prefix, pieces) in [("lower", lower_webbing), ("upper", upper_webbing)] {
        for (index, piece) in pieces.iter().enumerate() {
            assembly.push(ThreeMfObject {
                name: format!("webbing-{prefix}-{:02}", index + 1),
                solid: piece,
            });
        }
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
