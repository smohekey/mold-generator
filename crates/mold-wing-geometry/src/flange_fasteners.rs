use manifold_rust::{
    linalg::{Mat3x4, Vec3},
    manifold::Manifold,
    types::Error as ManifoldError,
};

use crate::{
    WingError, WingSpec, WingSurface, add_scaled, cross_array, dot, normalize_array, subtract,
    transverse_section_normal, wing_surface_frame,
};

#[derive(Debug, Clone, PartialEq)]
pub struct TransverseFlangeFastenerSpec {
    pub chord_fractions: Vec<f64>,
    pub clearance_diameter: f64,
    pub pilot_diameter: f64,
    pub pilot_depth: f64,
    pub cutter_overtravel: f64,
    pub circular_segments: i32,
}

impl Default for TransverseFlangeFastenerSpec {
    fn default() -> Self {
        Self {
            chord_fractions: vec![0.25, 0.75],
            clearance_diameter: 3.4,
            pilot_diameter: 2.5,
            pilot_depth: 4.0,
            cutter_overtravel: 0.5,
            circular_segments: 16,
        }
    }
}

pub struct TransverseFlangeFastenerCutters {
    pub surface: WingSurface,
    pub clearance: Manifold,
    pub pilot: Manifold,
}

#[allow(clippy::too_many_arguments)]
pub fn transverse_flange_fastener_cutters(
    wing: &WingSpec,
    seam_span: f64,
    shell_thickness: f64,
    outer_margin: f64,
    ramp_length: f64,
    bed_thickness: f64,
    fasteners: &TransverseFlangeFastenerSpec,
) -> Result<Vec<TransverseFlangeFastenerCutters>, WingError> {
    validate(
        shell_thickness,
        outer_margin,
        ramp_length,
        bed_thickness,
        fasteners,
    )?;
    let seam_normal = transverse_section_normal(wing, seam_span)?;
    let radial_offset = (shell_thickness + outer_margin) * 0.5;
    let mut cutters = Vec::with_capacity(fasteners.chord_fractions.len() * 2);

    for surface in [WingSurface::Lower, WingSurface::Upper] {
        for &chord_fraction in &fasteners.chord_fractions {
            let frame = wing_surface_frame(wing, seam_span, chord_fraction, surface)?;
            let radial_direction = normalize_array(add_scaled(
                frame.outward_normal,
                seam_normal,
                -dot(frame.outward_normal, seam_normal),
            ))
            .ok_or(WingError::InvalidSpec(
                "cannot project fastener placement into the seam plane",
            ))?;
            let interface = add_scaled(frame.point, radial_direction, radial_offset);
            let clearance_start = add_scaled(interface, seam_normal, -fasteners.cutter_overtravel);
            let clearance_end = add_scaled(
                interface,
                seam_normal,
                bed_thickness + fasteners.cutter_overtravel,
            );
            let pilot_start = add_scaled(interface, seam_normal, fasteners.cutter_overtravel);
            let pilot_end = add_scaled(interface, seam_normal, -fasteners.pilot_depth);
            cutters.push(TransverseFlangeFastenerCutters {
                surface,
                clearance: oriented_cylinder(
                    clearance_start,
                    clearance_end,
                    fasteners.clearance_diameter,
                    fasteners.circular_segments,
                )?,
                pilot: oriented_cylinder(
                    pilot_start,
                    pilot_end,
                    fasteners.pilot_diameter,
                    fasteners.circular_segments,
                )?,
            });
        }
    }
    Ok(cutters)
}

fn validate(
    shell_thickness: f64,
    outer_margin: f64,
    ramp_length: f64,
    bed_thickness: f64,
    fasteners: &TransverseFlangeFastenerSpec,
) -> Result<(), WingError> {
    if fasteners.chord_fractions.is_empty()
        || fasteners
            .chord_fractions
            .iter()
            .any(|fraction| !fraction.is_finite() || *fraction <= 0.0 || *fraction >= 1.0)
        || ![
            shell_thickness,
            outer_margin,
            ramp_length,
            bed_thickness,
            fasteners.clearance_diameter,
            fasteners.pilot_diameter,
            fasteners.pilot_depth,
            fasteners.cutter_overtravel,
        ]
        .into_iter()
        .all(f64::is_finite)
        || shell_thickness <= 0.0
        || outer_margin <= shell_thickness
        || ramp_length <= 0.0
        || bed_thickness <= 0.0
        || fasteners.clearance_diameter <= fasteners.pilot_diameter
        || fasteners.pilot_diameter <= 0.0
        || fasteners.pilot_depth <= 0.0
        || fasteners.pilot_depth >= ramp_length
        || fasteners.cutter_overtravel <= 0.0
        || fasteners.circular_segments < 8
    {
        return Err(WingError::InvalidSpec(
            "flange fasteners need safe dimensions, a through clearance hole, and a smaller blind pilot",
        ));
    }

    let center_offset = (shell_thickness + outer_margin) * 0.5;
    let pilot_end_margin =
        outer_margin - (outer_margin - shell_thickness) * fasteners.pilot_depth / ramp_length;
    if center_offset - fasteners.clearance_diameter * 0.5 <= shell_thickness
        || center_offset + fasteners.pilot_diameter * 0.5 >= pilot_end_margin
        || center_offset + fasteners.clearance_diameter * 0.5 >= outer_margin
    {
        return Err(WingError::InvalidSpec(
            "flange fastener holes would break through the ramp edge",
        ));
    }
    Ok(())
}

fn oriented_cylinder(
    start: [f64; 3],
    end: [f64; 3],
    diameter: f64,
    circular_segments: i32,
) -> Result<Manifold, WingError> {
    let displacement = subtract(end, start);
    let length = displacement
        .into_iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    let axis = normalize_array(displacement).ok_or(WingError::InvalidSpec(
        "flange fastener cutter needs a non-zero axis",
    ))?;
    let helper = if axis[2].abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let radial = normalize_array(cross_array(helper, axis)).ok_or(WingError::InvalidSpec(
        "cannot orient flange fastener cutter",
    ))?;
    let tangent = cross_array(axis, radial);
    let vector = |value: [f64; 3]| Vec3::new(value[0], value[1], value[2]);
    let transform = Mat3x4::from_cols(vector(radial), vector(tangent), vector(axis), vector(start));
    let cutter = Manifold::cylinder(length, diameter * 0.5, diameter * 0.5, circular_segments)
        .transform(&transform);
    if cutter.status() != ManifoldError::NoError {
        return Err(WingError::Geometry(cutter.status().to_string()));
    }
    Ok(cutter)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{preset, transverse_flange_blank};

    #[test]
    fn cutters_are_flange_centered_coaxial_and_stop_inside_the_ramp() {
        let wing = preset("tapered").unwrap();
        let fasteners = TransverseFlangeFastenerSpec::default();
        let cutters =
            transverse_flange_fastener_cutters(&wing, 200.0, 4.0, 15.2, 12.0, 3.0, &fasteners)
                .unwrap();

        assert_eq!(cutters.len(), 4);
        for (index, cutters) in cutters.into_iter().enumerate() {
            let clearance = cutters.clearance.bounding_box();
            let pilot = cutters.pilot.bounding_box();
            let chord_fraction = fasteners.chord_fractions[index % fasteners.chord_fractions.len()];
            let frame = wing_surface_frame(&wing, 200.0, chord_fraction, cutters.surface).unwrap();
            let seam_normal = transverse_section_normal(&wing, 200.0).unwrap();
            let radial_direction = normalize_array(add_scaled(
                frame.outward_normal,
                seam_normal,
                -dot(frame.outward_normal, seam_normal),
            ))
            .unwrap();
            let clearance_center = [
                (clearance.min.x + clearance.max.x) * 0.5,
                (clearance.min.y + clearance.max.y) * 0.5,
                (clearance.min.z + clearance.max.z) * 0.5,
            ];
            assert!((clearance.min.y - 199.5).abs() < 1.0e-9, "{clearance:?}");
            assert!((clearance.max.y - 203.5).abs() < 1.0e-9, "{clearance:?}");
            assert!((pilot.min.y - 196.0).abs() < 1.0e-9, "{pilot:?}");
            assert!((pilot.max.y - 200.5).abs() < 1.0e-9, "{pilot:?}");
            assert!(
                (dot(subtract(clearance_center, frame.point), radial_direction) - 9.6).abs()
                    < 1.0e-9
            );
            assert!(
                ((clearance.min.x + clearance.max.x) * 0.5 - (pilot.min.x + pilot.max.x) * 0.5)
                    .abs()
                    < 1.0e-9
            );
            assert!(
                ((clearance.min.z + clearance.max.z) * 0.5 - (pilot.min.z + pilot.max.z) * 0.5)
                    .abs()
                    < 1.0e-9
            );
        }
    }

    #[test]
    fn cutters_intersect_their_flange_without_reaching_the_ramp_start() {
        let wing = preset("tapered").unwrap();
        let fasteners = TransverseFlangeFastenerSpec::default();
        let cutters =
            transverse_flange_fastener_cutters(&wing, 200.0, 4.0, 15.2, 12.0, 3.0, &fasteners)
                .unwrap();
        let top = transverse_flange_blank(&wing, &[(188.0, 4.0), (200.0, 15.2)]).unwrap();
        let bed = transverse_flange_blank(&wing, &[(200.0, 15.2), (203.0, 15.2)]).unwrap();

        for cutters in cutters {
            assert!(top.intersection(&cutters.pilot).volume() > 0.0);
            assert!(bed.intersection(&cutters.clearance).volume() > 0.0);
            assert!(cutters.pilot.bounding_box().min.y > 188.0);
        }
    }

    #[test]
    fn flange_fasteners_reject_a_pilot_that_breaks_through_the_ramp() {
        let wing = preset("tapered").unwrap();
        let fasteners = TransverseFlangeFastenerSpec {
            pilot_depth: 10.0,
            ..TransverseFlangeFastenerSpec::default()
        };

        assert!(
            transverse_flange_fastener_cutters(&wing, 200.0, 4.0, 15.2, 12.0, 3.0, &fasteners,)
                .is_err()
        );
    }
}
