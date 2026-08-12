use manifold_rust::{manifold::Manifold, types::MeshGL64};

use crate::{
    FlangeFastenerBand, WingError, WingSpec, WingSurface, add_scaled, checked_mesh,
    connect_quad_rings, cross_array, dot, interpolate_station_extended, normalize_array,
    transverse_section_normal, wing_surface_frame,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WingTransverseRegistrationSettings {
    /// Number of fixtures placed on each independently printable flange constituent.
    pub fixtures_per_flange: usize,
    pub radial_half_width: f64,
    pub tangent_half_width: f64,
    pub span_half_depth: f64,
}

impl Default for WingTransverseRegistrationSettings {
    fn default() -> Self {
        Self {
            fixtures_per_flange: 2,
            radial_half_width: 5.0,
            tangent_half_width: 5.0,
            span_half_depth: 2.25,
        }
    }
}

/// Places registration inserts on one constituent transverse flange.
///
/// The chord range identifies one printable chord region and `surface` identifies
/// its lower or upper mold half. Each fixture is kept inside its share of the
/// tile and clear of the supplied fastener heads.
#[allow(clippy::too_many_arguments)]
pub fn transverse_flange_registration_inserts(
    wing: &WingSpec,
    center_span: f64,
    surface: WingSurface,
    chord_range: (f64, f64),
    band: FlangeFastenerBand,
    fastener_chord_fractions: &[f64],
    fastener_head_diameter: f64,
    settings: WingTransverseRegistrationSettings,
) -> Result<Vec<Manifold>, WingError> {
    validate(
        center_span,
        chord_range,
        band,
        fastener_chord_fractions,
        fastener_head_diameter,
        settings,
    )?;
    let station = interpolate_station_extended(wing, center_span)?;
    registration_chord_fractions(
        station.chord,
        chord_range,
        fastener_chord_fractions,
        fastener_head_diameter,
        settings,
    )?
    .into_iter()
    .map(|chord_fraction| {
        surface_registration_diamond(wing, center_span, chord_fraction, surface, band, settings)
    })
    .collect()
}

fn registration_chord_fractions(
    chord: f64,
    chord_range: (f64, f64),
    fastener_chord_fractions: &[f64],
    fastener_head_diameter: f64,
    settings: WingTransverseRegistrationSettings,
) -> Result<Vec<f64>, WingError> {
    let footprint_half_width = settings.radial_half_width.max(settings.tangent_half_width);
    let tile_start = chord * chord_range.0;
    let tile_end = chord * chord_range.1;
    let lane_width = (tile_end - tile_start) / settings.fixtures_per_flange as f64;
    if lane_width <= footprint_half_width * 2.0 {
        return Err(WingError::InvalidSpec(
            "transverse flange constituent is too narrow for its registration fixtures",
        ));
    }

    let fastener_clearance = footprint_half_width + fastener_head_diameter * 0.5;
    let fastener_positions: Vec<f64> = fastener_chord_fractions
        .iter()
        .map(|fraction| fraction * chord)
        .collect();
    let mut positions = Vec::with_capacity(settings.fixtures_per_flange);
    for index in 0..settings.fixtures_per_flange {
        let lane_start = tile_start + lane_width * index as f64 + footprint_half_width;
        let lane_end = tile_start + lane_width * (index + 1) as f64 - footprint_half_width;
        let available = subtract_blocked_intervals(
            (lane_start, lane_end),
            &fastener_positions,
            fastener_clearance,
        );
        let position = available
            .into_iter()
            .max_by(|a, b| interval_length(*a).total_cmp(&interval_length(*b)))
            .filter(|interval| interval_length(*interval) > 1.0e-6)
            .map(|interval| (interval.0 + interval.1) * 0.5)
            .ok_or(WingError::InvalidSpec(
                "transverse flange has no registration position clear of its fasteners",
            ))?;
        positions.push(position / chord);
    }
    Ok(positions)
}

fn validate(
    center_span: f64,
    chord_range: (f64, f64),
    band: FlangeFastenerBand,
    fastener_chord_fractions: &[f64],
    fastener_head_diameter: f64,
    settings: WingTransverseRegistrationSettings,
) -> Result<(), WingError> {
    if ![
        center_span,
        chord_range.0,
        chord_range.1,
        band.inner_margin,
        band.outer_margin,
        fastener_head_diameter,
        settings.radial_half_width,
        settings.tangent_half_width,
        settings.span_half_depth,
    ]
    .into_iter()
    .all(f64::is_finite)
        || chord_range.0 < 0.0
        || chord_range.1 > 1.0
        || chord_range.1 <= chord_range.0
        || band.inner_margin < 0.0
        || band.outer_margin <= band.inner_margin
        || fastener_head_diameter <= 0.0
        || settings.fixtures_per_flange == 0
        || settings.radial_half_width <= 0.0
        || settings.tangent_half_width <= 0.0
        || settings.span_half_depth <= 0.0
        || settings.radial_half_width * 2.0 > band.outer_margin - band.inner_margin
        || fastener_chord_fractions
            .iter()
            .any(|fraction| !fraction.is_finite() || !(0.0..=1.0).contains(fraction))
    {
        return Err(WingError::InvalidSpec(
            "transverse registration settings and flange range must be valid",
        ));
    }
    Ok(())
}

fn subtract_blocked_intervals(
    available: (f64, f64),
    obstacles: &[f64],
    clearance: f64,
) -> Vec<(f64, f64)> {
    let mut intervals = vec![available];
    for obstacle in obstacles {
        let blocked = (obstacle - clearance, obstacle + clearance);
        intervals = intervals
            .into_iter()
            .flat_map(|interval| subtract_interval(interval, blocked))
            .collect();
    }
    intervals
}

fn subtract_interval(available: (f64, f64), blocked: (f64, f64)) -> Vec<(f64, f64)> {
    if blocked.1 <= available.0 || blocked.0 >= available.1 {
        return vec![available];
    }
    let mut remainder = Vec::with_capacity(2);
    if blocked.0 > available.0 {
        remainder.push((available.0, blocked.0.min(available.1)));
    }
    if blocked.1 < available.1 {
        remainder.push((blocked.1.max(available.0), available.1));
    }
    remainder
}

fn interval_length(interval: (f64, f64)) -> f64 {
    interval.1 - interval.0
}

fn surface_registration_diamond(
    wing: &WingSpec,
    center_span: f64,
    chord_fraction: f64,
    surface: WingSurface,
    band: FlangeFastenerBand,
    settings: WingTransverseRegistrationSettings,
) -> Result<Manifold, WingError> {
    let frame = wing_surface_frame(wing, center_span, chord_fraction, surface)?;
    let seam_normal = transverse_section_normal(wing, center_span)?;
    let radial = normalize_array(add_scaled(
        frame.outward_normal,
        seam_normal,
        -dot(frame.outward_normal, seam_normal),
    ))
    .ok_or(WingError::InvalidSpec(
        "cannot project transverse registration placement into the flange plane",
    ))?;
    let tangent = normalize_array(cross_array(radial, seam_normal)).ok_or(
        WingError::InvalidSpec("cannot determine transverse registration tangent"),
    )?;
    let center = add_scaled(
        frame.point,
        radial,
        (band.inner_margin + band.outer_margin) * 0.5,
    );
    let footprint = [
        add_scaled(center, radial, -settings.radial_half_width),
        add_scaled(center, tangent, settings.tangent_half_width),
        add_scaled(center, radial, settings.radial_half_width),
        add_scaled(center, tangent, -settings.tangent_half_width),
    ];
    let base = footprint.map(|point| add_scaled(point, seam_normal, -settings.span_half_depth));
    let top = footprint.map(|point| add_scaled(point, seam_normal, settings.span_half_depth));
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
    checked_mesh(mesh, "transverse registration diamond")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        WingBaseAttachmentSettings, WingFlangeFastenerSpec, generate, preset,
        transverse_through_flange_fastener_cutters, wing_base_attachment_geometry,
    };

    #[test]
    fn gull_root_gets_two_fixtures_per_surface_and_chord_tile() {
        let wing = preset("gull").unwrap();
        let part = generate(&wing).unwrap();
        let base = wing_base_attachment_geometry(
            &wing,
            &part,
            WingBaseAttachmentSettings {
                flange_width: 14.4,
                axial_thickness: 3.0,
            },
        )
        .unwrap();
        let fasteners = WingFlangeFastenerSpec::default();
        let cutters =
            transverse_through_flange_fastener_cutters(&wing, 0.0, 4.0, 14.4, 3.0, 3.0, &fasteners)
                .unwrap();
        let settings = WingTransverseRegistrationSettings::default();
        let band = FlangeFastenerBand {
            inner_margin: 4.0,
            outer_margin: 14.4,
        };
        let mut count = 0;

        for chord_range in [(0.0, 0.5), (0.5, 1.0)] {
            for surface in [WingSurface::Lower, WingSurface::Upper] {
                let positions: Vec<f64> = cutters
                    .iter()
                    .filter(|cutter| cutter.surface == surface)
                    .map(|cutter| cutter.chord_fraction)
                    .collect();
                let inserts = transverse_flange_registration_inserts(
                    &wing,
                    0.0,
                    surface,
                    chord_range,
                    band,
                    &positions,
                    fasteners.head_diameter,
                    settings,
                )
                .unwrap();

                assert_eq!(inserts.len(), 2);
                let chord_fractions = registration_chord_fractions(
                    interpolate_station_extended(&wing, 0.0).unwrap().chord,
                    chord_range,
                    &positions,
                    fasteners.head_diameter,
                    settings,
                )
                .unwrap();
                assert!(
                    chord_fractions
                        .iter()
                        .all(|position| *position > chord_range.0 && *position < chord_range.1)
                );
                for insert in inserts {
                    assert!(!insert.is_empty());
                    assert!(base.mold_flange.intersection(&insert).volume() > 1.0e-9);
                    assert!(base.sealing_profile.intersection(&insert).volume() > 1.0e-9);
                    let bounds = insert.bounding_box();
                    match surface {
                        WingSurface::Lower => assert!(bounds.max.z < 0.0),
                        WingSurface::Upper => assert!(bounds.min.z > 0.0),
                    }
                    for cutter in cutters.iter().filter(|cutter| cutter.surface == surface) {
                        assert!(insert.intersection(&cutter.cutter).volume() < 1.0e-9);
                    }
                    count += 1;
                }
            }
        }

        assert_eq!(count, 8);
    }

    #[test]
    fn fixture_placement_fails_instead_of_silently_omitting_a_constituent_flange() {
        let wing = preset("rectangular").unwrap();
        let result = transverse_flange_registration_inserts(
            &wing,
            0.0,
            WingSurface::Upper,
            (0.0, 0.05),
            FlangeFastenerBand {
                inner_margin: 4.0,
                outer_margin: 14.4,
            },
            &[],
            6.0,
            WingTransverseRegistrationSettings::default(),
        );

        assert!(matches!(result, Err(WingError::InvalidSpec(_))));
    }
}
