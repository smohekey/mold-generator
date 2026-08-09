use mold_geometry::{Bounds3, SolidKernel, Vec3};
use mold_manifold::ManifoldKernel;

#[test]
fn offset_expands_a_cube_by_requested_distance() {
    let kernel = ManifoldKernel;
    let cube = kernel
        .cuboid(Bounds3 {
            min: Vec3::new(-10.0, -20.0, -30.0),
            max: Vec3::new(10.0, 20.0, 30.0),
        })
        .unwrap();

    let expanded = kernel.offset(&cube, 2.0).unwrap();
    let bounds = kernel.bounds(&expanded).unwrap();

    assert_close(bounds.min.x, -12.0);
    assert_close(bounds.min.y, -22.0);
    assert_close(bounds.min.z, -32.0);
    assert_close(bounds.max.x, 12.0);
    assert_close(bounds.max.y, 22.0);
    assert_close(bounds.max.z, 32.0);
    assert!(expanded.0.volume() > cube.0.volume());
}

#[test]
fn offset_rejects_non_positive_distances() {
    let kernel = ManifoldKernel;
    let cube = kernel
        .cuboid(Bounds3 {
            min: Vec3::new(0.0, 0.0, 0.0),
            max: Vec3::new(1.0, 1.0, 1.0),
        })
        .unwrap();

    assert!(kernel.offset(&cube, 0.0).is_err());
    assert!(kernel.offset(&cube, -1.0).is_err());
}

fn assert_close(actual: f64, expected: f64) {
    let tolerance = 1.0e-8 * expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual}"
    );
}
