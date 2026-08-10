use std::num::NonZeroUsize;

use mold_core::Axis;
use mold_geometry::{SolidKernel, Vec3};
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{
    PrintVolume, SegmentBoundary, SegmentationSettings, ShellSettings, TiledSegmentationSettings,
    generate_sectioned_shell_mold, partition_for_print_volume, partition_tiles_for_print_volume,
};
use mold_wing_geometry::{
    PrintableEnvelope, generate, preset, printable_segment_dimensions, printable_tile_dimensions,
    wing_segment_boundaries,
};

#[test]
fn gull_wing_generates_valid_sectioned_shell_mold() {
    let kernel = ManifoldKernel;
    let wing = ManifoldSolid(generate(&preset("gull").unwrap()).unwrap());
    let part_bounds = kernel.bounds(&wing).unwrap();
    let settings = ShellSettings {
        thickness: 3.0,
        flange_width: 12.0,
        web_thickness: 3.0,
        structural_webbing: Some(Default::default()),
    };

    let mold = generate_sectioned_shell_mold(
        &kernel,
        &wing,
        Axis::Z,
        Axis::Y,
        NonZeroUsize::new(2).unwrap(),
        settings,
    )
    .unwrap();

    assert_eq!(mold.negative.len(), 2);
    assert_eq!(mold.positive.len(), 2);

    let split = (part_bounds.min.z + part_bounds.max.z) * 0.5;
    for section in &mold.negative {
        assert_eq!(section.0.status().to_str(), "No Error");
        assert!(!section.0.is_empty());
        assert!(section.0.volume() > 0.0);
        let bounds = kernel.bounds(section).unwrap();
        assert!(bounds.max.z <= split + settings.thickness + settings.flange_width);
    }

    for section in &mold.positive {
        assert_eq!(section.0.status().to_str(), "No Error");
        assert!(!section.0.is_empty());
        assert!(section.0.volume() > 0.0);
        let bounds = kernel.bounds(section).unwrap();
        assert!(bounds.min.z >= split - settings.thickness - settings.flange_width);
    }

    // The shell should extend outside the source wing in every dimension,
    // while remaining far smaller than the previous full rectangular blank.
    let all_bounds = mold
        .negative
        .iter()
        .chain(&mold.positive)
        .map(|section| kernel.bounds(section).unwrap())
        .fold(None, |acc: Option<(Vec3, Vec3)>, bounds| {
            Some(match acc {
                None => (bounds.min, bounds.max),
                Some((min, max)) => (
                    Vec3::new(
                        min.x.min(bounds.min.x),
                        min.y.min(bounds.min.y),
                        min.z.min(bounds.min.z),
                    ),
                    Vec3::new(
                        max.x.max(bounds.max.x),
                        max.y.max(bounds.max.y),
                        max.z.max(bounds.max.z),
                    ),
                ),
            })
        })
        .unwrap();

    assert!(all_bounds.0.x < part_bounds.min.x);
    assert!(all_bounds.1.x > part_bounds.max.x);
    assert!(all_bounds.0.z < part_bounds.min.z);
    assert!(all_bounds.1.z > part_bounds.max.z);
}

#[test]
fn extrusion_width_controls_structural_web_thickness() {
    let settings = mold_shell::WebbingSettings {
        extrusion_width: 0.48,
        wall_line_count: 3,
        ..Default::default()
    };
    assert!((settings.thickness() - 1.44).abs() < 1.0e-9);
}

#[test]
fn print_volume_constrains_gull_wing_segmentation_and_preserves_the_bend() {
    let spec = preset("gull").unwrap();
    let candidates = wing_segment_boundaries(&spec, -3.0, 603.0, 25.0).unwrap();
    let boundaries: Vec<SegmentBoundary> = candidates
        .iter()
        .map(|candidate| SegmentBoundary {
            position: candidate.position,
            preference: candidate.deviation,
        })
        .collect();
    let envelope = PrintableEnvelope {
        flange_margin: 12.0,
        shell_thickness: 3.0,
        web_depth: 8.0,
        span_samples: 24,
    };
    let partition = |print_volume| {
        partition_for_print_volume(
            &boundaries,
            SegmentationSettings {
                print_volume,
                preferred_segment_count: None,
                max_segment_count: 8,
            },
            |start, end| printable_segment_dimensions(&spec, start, end, envelope).ok(),
        )
        .unwrap()
    };

    let tall_volume = PrintVolume {
        width: 320.0,
        depth: 320.0,
        height: 500.0,
        clearance: 5.0,
    };
    let short_volume = PrintVolume {
        height: 300.0,
        ..tall_volume
    };
    let compact_volume = PrintVolume {
        width: 256.0,
        depth: 256.0,
        height: 256.0,
        clearance: 6.0,
    };
    let tall_printer = partition(tall_volume);
    let short_printer = partition(short_volume);
    let compact_printer = partition(compact_volume);

    assert_eq!(tall_printer, vec![(-3.0, 180.0), (180.0, 603.0)]);
    assert!(short_printer.len() > tall_printer.len());
    for range in short_printer {
        let dimensions = printable_segment_dimensions(&spec, range.0, range.1, envelope).unwrap();
        assert!(short_volume.fits(dimensions));
    }
    assert_eq!(compact_printer.len(), 3);
    for range in compact_printer {
        let dimensions = printable_segment_dimensions(&spec, range.0, range.1, envelope).unwrap();
        assert!(compact_volume.fits(dimensions));
    }
}

#[test]
fn narrow_printer_uses_longitudinal_tiles_when_rotation_cannot_fit() {
    let spec = preset("gull").unwrap();
    let candidates = wing_segment_boundaries(&spec, -3.0, 603.0, 25.0).unwrap();
    let boundaries: Vec<SegmentBoundary> = candidates
        .iter()
        .map(|candidate| SegmentBoundary {
            position: candidate.position,
            preference: candidate.deviation,
        })
        .collect();
    let envelope = PrintableEnvelope {
        flange_margin: 12.0,
        shell_thickness: 3.0,
        web_depth: 8.0,
        span_samples: 24,
    };
    let volume = PrintVolume {
        width: 160.0,
        depth: 160.0,
        height: 300.0,
        clearance: 6.0,
    };
    let tiles = partition_tiles_for_print_volume(
        &boundaries,
        TiledSegmentationSettings {
            span: SegmentationSettings {
                print_volume: volume,
                preferred_segment_count: None,
                max_segment_count: 8,
            },
            max_longitudinal_segments: 4,
        },
        |start, end, chord| printable_tile_dimensions(&spec, start, end, chord, envelope).ok(),
    )
    .unwrap();

    assert!(tiles.iter().any(|tile| tile.chord != (0.0, 1.0)));
    for tile in tiles {
        let dimensions =
            printable_tile_dimensions(&spec, tile.span.0, tile.span.1, tile.chord, envelope)
                .unwrap();
        assert!(volume.fits(dimensions));
    }
}
