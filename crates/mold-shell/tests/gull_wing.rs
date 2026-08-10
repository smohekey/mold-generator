use std::num::NonZeroUsize;

use mold_core::Axis;
use mold_geometry::{SolidKernel, Vec3};
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{ShellSettings, generate_sectioned_shell_mold};
use mold_test_models::{generate, preset};

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
