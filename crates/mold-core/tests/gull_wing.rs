use std::num::NonZeroUsize;

use mold_core::{
    Axis, MoldSettings, SectionRegistration, generate_registered_sectioned_two_part_mold,
    generate_sectioned_two_part_mold,
};
use mold_geometry::{SolidKernel, Vec3};
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_test_models::{generate, preset};

#[test]
fn gull_wing_generates_valid_sectioned_two_part_mold() {
    let kernel = ManifoldKernel;
    let wing = gull_wing();
    let part_bounds = kernel.bounds(&wing).unwrap();

    let section_count = NonZeroUsize::new(4).unwrap();
    let margin = Vec3::new(10.0, 10.0, 10.0);
    let mold = generate_sectioned_two_part_mold(
        &kernel,
        &wing,
        MoldSettings { margin },
        Axis::Z,
        Axis::Y,
        section_count,
    )
    .unwrap();

    assert_eq!(mold.negative.len(), section_count.get());
    assert_eq!(mold.positive.len(), section_count.get());

    let expected_min_y = part_bounds.min.y - margin.y;
    let expected_max_y = part_bounds.max.y + margin.y;
    let expected_width = (expected_max_y - expected_min_y) / section_count.get() as f64;
    let split_z = (part_bounds.min.z + part_bounds.max.z) * 0.5;

    for (index, section) in mold.negative.iter().enumerate() {
        assert_valid(section);
        let bounds = kernel.bounds(section).unwrap();
        let (expected_min, expected_max) = expected_section_range(
            index,
            section_count.get(),
            expected_min_y,
            expected_max_y,
            expected_width,
        );

        assert_close(bounds.min.y, expected_min);
        assert_close(bounds.max.y, expected_max);
        assert!(bounds.max.z <= split_z + 1.0e-8);
    }

    for (index, section) in mold.positive.iter().enumerate() {
        assert_valid(section);
        let bounds = kernel.bounds(section).unwrap();
        let (expected_min, expected_max) = expected_section_range(
            index,
            section_count.get(),
            expected_min_y,
            expected_max_y,
            expected_width,
        );

        assert_close(bounds.min.y, expected_min);
        assert_close(bounds.max.y, expected_max);
        assert!(bounds.min.z >= split_z - 1.0e-8);
    }
}

#[test]
fn gull_wing_registered_sections_have_mating_keys() {
    let kernel = ManifoldKernel;
    let wing = gull_wing();
    let part_bounds = kernel.bounds(&wing).unwrap();
    let section_count = NonZeroUsize::new(4).unwrap();
    let margin = Vec3::new(10.0, 10.0, 10.0);
    let registration = SectionRegistration::default();

    let mold = generate_registered_sectioned_two_part_mold(
        &kernel,
        &wing,
        MoldSettings { margin },
        Axis::Z,
        Axis::Y,
        section_count,
        registration,
    )
    .unwrap();

    let expected_min_y = part_bounds.min.y - margin.y;
    let expected_max_y = part_bounds.max.y + margin.y;
    let expected_width = (expected_max_y - expected_min_y) / section_count.get() as f64;

    for half in [&mold.negative, &mold.positive] {
        assert_eq!(half.len(), section_count.get());
        for (index, section) in half.iter().enumerate() {
            assert_valid(section);
            let bounds = kernel.bounds(section).unwrap();
            let (expected_min, expected_max) = expected_section_range(
                index,
                section_count.get(),
                expected_min_y,
                expected_max_y,
                expected_width,
            );

            assert_close(bounds.min.y, expected_min);
            if index + 1 == section_count.get() {
                assert_close(bounds.max.y, expected_max);
            } else {
                assert_close(bounds.max.y, expected_max + registration.depth);
            }
        }
    }
}

fn gull_wing() -> ManifoldSolid {
    ManifoldSolid(generate(&preset("gull").unwrap()).unwrap())
}

fn expected_section_range(
    index: usize,
    count: usize,
    min: f64,
    max: f64,
    width: f64,
) -> (f64, f64) {
    let section_min = min + index as f64 * width;
    let section_max = if index + 1 == count {
        max
    } else {
        min + (index + 1) as f64 * width
    };
    (section_min, section_max)
}

fn assert_valid(section: &ManifoldSolid) {
    assert_eq!(section.0.status().to_str(), "No Error");
    assert!(!section.0.is_empty());
    assert!(section.0.volume() > 0.0);
}

fn assert_close(actual: f64, expected: f64) {
    let tolerance = 1.0e-8 * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual}"
    );
}
