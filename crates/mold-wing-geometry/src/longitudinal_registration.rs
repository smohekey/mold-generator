use manifold_rust::{manifold::Manifold, types::MeshGL64};

use crate::{
    FlangeFastenerBand, WingError, WingSpec, WingSurface, add_scaled, checked_mesh,
    connect_quad_rings, cross_array, normalize_array, subtract, wing_surface_frame,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WingLongitudinalRegistrationSettings {
    pub radial_half_width: f64,
    pub span_half_width: f64,
    pub seam_half_depth: f64,
}

impl Default for WingLongitudinalRegistrationSettings {
    fn default() -> Self {
        Self {
            radial_half_width: 5.0,
            span_half_width: 7.0,
            seam_half_depth: 2.25,
        }
    }
}

/// Places registration inserts on a longitudinal flange shared by adjacent
/// chord tiles. One insert is centered in each sufficiently wide gap between
/// fasteners, and generation fails if the flange cannot accommodate any.
#[allow(clippy::too_many_arguments)]
pub fn longitudinal_split_flange_registration_inserts(
    wing: &WingSpec,
    chord_fraction: f64,
    span_range: (f64, f64),
    surface: WingSurface,
    outward_direction: [f64; 3],
    band: FlangeFastenerBand,
    fastener_positions: &[f64],
    fastener_head_diameter: f64,
    settings: WingLongitudinalRegistrationSettings,
) -> Result<Vec<Manifold>, WingError> {
    validate(
        chord_fraction,
        span_range,
        band,
        fastener_positions,
        fastener_head_diameter,
        settings,
    )?;
    let outward_direction = normalize_array(outward_direction).ok_or(WingError::InvalidSpec(
        "longitudinal registration needs a non-zero outward direction",
    ))?;
    let clearance = settings.span_half_width + fastener_head_diameter * 0.5;
    let positions: Vec<f64> = fastener_positions
        .windows(2)
        .filter(|pair| pair[1] - pair[0] > clearance * 2.0)
        .map(|pair| (pair[0] + pair[1]) * 0.5)
        .collect();
    if positions.is_empty() {
        return Err(WingError::InvalidSpec(
            "longitudinal flange has no registration position clear of its fasteners",
        ));
    }

    positions
        .into_iter()
        .map(|position| {
            surface_registration_diamond(
                wing,
                chord_fraction,
                span_range,
                position,
                surface,
                outward_direction,
                band,
                settings,
            )
        })
        .collect()
}

fn validate(
    chord_fraction: f64,
    span_range: (f64, f64),
    band: FlangeFastenerBand,
    fastener_positions: &[f64],
    fastener_head_diameter: f64,
    settings: WingLongitudinalRegistrationSettings,
) -> Result<(), WingError> {
    if ![
        chord_fraction,
        span_range.0,
        span_range.1,
        band.inner_margin,
        band.outer_margin,
        fastener_head_diameter,
        settings.radial_half_width,
        settings.span_half_width,
        settings.seam_half_depth,
    ]
    .into_iter()
    .all(f64::is_finite)
        || !(0.0..1.0).contains(&chord_fraction)
        || span_range.1 <= span_range.0
        || band.inner_margin < 0.0
        || band.outer_margin <= band.inner_margin
        || fastener_head_diameter <= 0.0
        || settings.radial_half_width <= 0.0
        || settings.span_half_width <= 0.0
        || settings.seam_half_depth <= 0.0
        || settings.radial_half_width * 2.0 > band.outer_margin - band.inner_margin
        || fastener_positions.len() < 2
        || fastener_positions.iter().any(|position| {
            !position.is_finite() || *position < span_range.0 || *position > span_range.1
        })
        || fastener_positions.windows(2).any(|pair| pair[1] <= pair[0])
    {
        return Err(WingError::InvalidSpec(
            "longitudinal registration settings and fastener positions must be valid",
        ));
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn surface_registration_diamond(
    wing: &WingSpec,
    chord_fraction: f64,
    span_range: (f64, f64),
    center_span: f64,
    surface: WingSurface,
    radial: [f64; 3],
    band: FlangeFastenerBand,
    settings: WingLongitudinalRegistrationSettings,
) -> Result<Manifold, WingError> {
    let frame = wing_surface_frame(wing, center_span, chord_fraction, surface)?;
    let span_step = ((span_range.1 - span_range.0) * 1.0e-4).max(1.0e-4);
    let before = wing_surface_frame(
        wing,
        (center_span - span_step).max(span_range.0),
        chord_fraction,
        surface,
    )?;
    let after = wing_surface_frame(
        wing,
        (center_span + span_step).min(span_range.1),
        chord_fraction,
        surface,
    )?;
    let tangent = normalize_array(subtract(after.point, before.point)).ok_or(
        WingError::InvalidSpec("cannot determine longitudinal registration tangent"),
    )?;
    let seam_normal = normalize_array(cross_array(tangent, radial)).ok_or(
        WingError::InvalidSpec("cannot determine longitudinal registration seam normal"),
    )?;
    let center = add_scaled(
        frame.point,
        radial,
        (band.inner_margin + band.outer_margin) * 0.5,
    );
    let footprint = [
        add_scaled(center, radial, -settings.radial_half_width),
        add_scaled(center, tangent, settings.span_half_width),
        add_scaled(center, radial, settings.radial_half_width),
        add_scaled(center, tangent, -settings.span_half_width),
    ];
    let base = footprint.map(|point| add_scaled(point, seam_normal, -settings.seam_half_depth));
    let top = footprint.map(|point| add_scaled(point, seam_normal, settings.seam_half_depth));
    let mut mesh = MeshGL64 {
        num_prop: 3,
        ..Default::default()
    };
    for point in base.into_iter().chain(top) {
        mesh.vert_properties.extend(point);
    }
    connect_quad_rings(&mut mesh, 0, 4);
    mesh.tri_verts.extend([0, 2, 1, 0, 3, 2]);
    mesh.tri_verts.extend([4, 5, 6, 4, 6, 7]);
    checked_mesh(mesh, "longitudinal registration diamond")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        FlangeEndObstructions, WingFlangeFastenerSpec, longitudinal_split_flange_fastener_cutters,
        preset, segment_normal,
    };

    #[test]
    fn split_flange_registrations_follow_each_surface_and_clear_fasteners() {
        let wing = preset("gull").unwrap();
        let range = (0.0, 216.0);
        let band = FlangeFastenerBand {
            inner_margin: 4.0,
            outer_margin: 14.4,
        };
        let fasteners = WingFlangeFastenerSpec::default();
        let upper_direction = segment_normal(&wing, range.0, range.1).unwrap();

        for (surface, direction) in [
            (WingSurface::Lower, upper_direction.map(|value| -value)),
            (WingSurface::Upper, upper_direction),
        ] {
            let cutters = longitudinal_split_flange_fastener_cutters(
                &wing,
                0.5,
                range,
                surface,
                direction,
                band,
                3.0,
                FlangeEndObstructions {
                    start: 0.0,
                    end: 12.0,
                },
                &fasteners,
            )
            .unwrap();
            let positions: Vec<f64> = cutters.iter().map(|cutter| cutter.position).collect();
            let inserts = longitudinal_split_flange_registration_inserts(
                &wing,
                0.5,
                range,
                surface,
                direction,
                band,
                &positions,
                fasteners.head_diameter,
                WingLongitudinalRegistrationSettings::default(),
            )
            .unwrap();

            assert!(!inserts.is_empty());
            for insert in inserts {
                assert!(!insert.is_empty());
                for cutter in &cutters {
                    assert!(insert.intersection(&cutter.cutter).volume() < 1.0e-9);
                }
            }
        }
    }

    #[test]
    fn split_flange_registration_fails_when_fasteners_leave_no_room() {
        let wing = preset("rectangular").unwrap();
        let result = longitudinal_split_flange_registration_inserts(
            &wing,
            0.5,
            (0.0, 100.0),
            WingSurface::Upper,
            [0.0, 0.0, 1.0],
            FlangeFastenerBand {
                inner_margin: 4.0,
                outer_margin: 14.4,
            },
            &[40.0, 55.0],
            6.0,
            WingLongitudinalRegistrationSettings::default(),
        );

        assert!(matches!(result, Err(WingError::InvalidSpec(_))));
    }
}
