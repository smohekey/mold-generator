use manifold_rust::{
    linalg::{Mat3x4, Vec3},
    manifold::Manifold,
    types::Error as ManifoldError,
};
use mold_geometry::EndAnchoredDistribution;

use crate::{
    WingEdge, WingError, WingSpec, WingSurface, add_scaled, cross_array, dot, flange_normal,
    interpolate_station, interpolate_station_extended, normalize_array, subtract,
    transform_station, transverse_section_normal, wing_surface_frame,
};

#[derive(Debug, Clone, PartialEq)]
pub enum WingFlangeFastenerLayout {
    /// Anchor fasteners near both ends, then distribute interior fasteners
    /// evenly so no spacing exceeds this distance.
    MaximumSpacing(f64),
    /// Place fasteners at explicit normalized positions along each flange.
    Fractions(Vec<f64>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct WingFlangeFastenerSpec {
    /// Layout applied independently along every mating flange pair.
    pub layout: WingFlangeFastenerLayout,
    /// Screw-head diameter used to keep end fasteners two head diameters from each end.
    pub head_diameter: f64,
    pub clearance_diameter: f64,
    pub pilot_diameter: f64,
    pub pilot_depth: f64,
    pub cutter_overtravel: f64,
    pub circular_segments: i32,
}

/// Length occupied by adjoining geometry at each end of a longitudinal flange.
///
/// Automatic layouts keep the complete fastener head beyond each obstruction.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FlangeEndObstructions {
    pub start: f64,
    pub end: f64,
}

impl Default for WingFlangeFastenerSpec {
    fn default() -> Self {
        Self {
            layout: WingFlangeFastenerLayout::MaximumSpacing(100.0),
            head_diameter: 6.0,
            clearance_diameter: 3.4,
            pilot_diameter: 2.5,
            pilot_depth: 4.0,
            cutter_overtravel: 0.5,
            circular_segments: 16,
        }
    }
}

pub type TransverseFlangeFastenerLayout = WingFlangeFastenerLayout;
pub type TransverseFlangeFastenerSpec = WingFlangeFastenerSpec;

pub struct TransverseFlangeFastenerCutters {
    pub surface: WingSurface,
    pub clearance: Manifold,
    pub pilot: Manifold,
}

pub struct TransverseThroughFlangeFastenerCutter {
    pub surface: WingSurface,
    pub cutter: Manifold,
}

pub struct LongitudinalEdgeFastenerCutter {
    pub position: f64,
    pub cutter: Manifold,
}

pub struct LongitudinalSplitFlangeFastenerCutter {
    pub position: f64,
    pub cutter: Manifold,
}

struct TransverseFastenerInterfaces {
    seam_normal: [f64; 3],
    placements: Vec<(WingSurface, [f64; 3])>,
}

#[allow(clippy::too_many_arguments)]
pub fn transverse_flange_fastener_cutters(
    wing: &WingSpec,
    seam_span: f64,
    shell_thickness: f64,
    outer_margin: f64,
    ramp_length: f64,
    bed_thickness: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<TransverseFlangeFastenerCutters>, WingError> {
    validate(
        shell_thickness,
        outer_margin,
        ramp_length,
        bed_thickness,
        fasteners,
    )?;
    let TransverseFastenerInterfaces {
        seam_normal,
        placements,
    } = transverse_fastener_interfaces(wing, seam_span, shell_thickness, outer_margin, fasteners)?;
    let mut cutters = Vec::with_capacity(placements.len());

    for (surface, interface) in placements {
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
    Ok(cutters)
}

#[allow(clippy::too_many_arguments)]
pub fn transverse_through_flange_fastener_cutters(
    wing: &WingSpec,
    seam_span: f64,
    inner_margin: f64,
    outer_margin: f64,
    negative_depth: f64,
    positive_depth: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<TransverseThroughFlangeFastenerCutter>, WingError> {
    validate_through_flange(
        inner_margin,
        outer_margin,
        negative_depth,
        positive_depth,
        fasteners,
    )?;
    let TransverseFastenerInterfaces {
        seam_normal,
        placements,
    } = transverse_fastener_interfaces(wing, seam_span, inner_margin, outer_margin, fasteners)?;
    placements
        .into_iter()
        .map(|(surface, interface)| {
            let start = add_scaled(
                interface,
                seam_normal,
                -(negative_depth + fasteners.cutter_overtravel),
            );
            let end = add_scaled(
                interface,
                seam_normal,
                positive_depth + fasteners.cutter_overtravel,
            );
            Ok(TransverseThroughFlangeFastenerCutter {
                surface,
                cutter: oriented_cylinder(
                    start,
                    end,
                    fasteners.clearance_diameter,
                    fasteners.circular_segments,
                )?,
            })
        })
        .collect()
}

pub fn longitudinal_edge_fastener_cutters(
    wing: &WingSpec,
    edge: WingEdge,
    span_range: (f64, f64),
    flange_width: f64,
    half_depth: f64,
    end_obstructions: FlangeEndObstructions,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<LongitudinalEdgeFastenerCutter>, WingError> {
    validate_through_flange(0.0, flange_width, half_depth, half_depth, fasteners)?;
    fastener_positions_with_end_obstructions(span_range, end_obstructions, fasteners)?
        .into_iter()
        .map(|position| {
            let station = interpolate_station(wing, position)?;
            let center_x = match edge {
                WingEdge::Leading => -flange_width * 0.5,
                WingEdge::Trailing => station.chord + flange_width * 0.5,
            };
            let center = transform_station(&station, center_x, 0.0);
            let normal = flange_normal(wing, edge, position)?;
            let start = add_scaled(center, normal, -(half_depth + fasteners.cutter_overtravel));
            let end = add_scaled(center, normal, half_depth + fasteners.cutter_overtravel);
            Ok(LongitudinalEdgeFastenerCutter {
                position,
                cutter: oriented_cylinder(
                    start,
                    end,
                    fasteners.clearance_diameter,
                    fasteners.circular_segments,
                )?,
            })
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
pub fn longitudinal_split_flange_fastener_cutters(
    wing: &WingSpec,
    chord_fraction: f64,
    span_range: (f64, f64),
    surface: WingSurface,
    outward_direction: [f64; 3],
    attachment_offset: f64,
    flange_width: f64,
    flange_thickness: f64,
    end_obstructions: FlangeEndObstructions,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<LongitudinalSplitFlangeFastenerCutter>, WingError> {
    validate_through_flange(
        0.0,
        flange_width,
        flange_thickness * 0.5,
        flange_thickness * 0.5,
        fasteners,
    )?;
    if !chord_fraction.is_finite()
        || !(0.0..1.0).contains(&chord_fraction)
        || !attachment_offset.is_finite()
        || attachment_offset < 0.0
    {
        return Err(WingError::InvalidSpec(
            "invalid longitudinal split flange fastener placement",
        ));
    }
    let outward_direction = normalize_array(outward_direction).ok_or(WingError::InvalidSpec(
        "longitudinal split flange needs a non-zero outward direction",
    ))?;
    let span_step = ((span_range.1 - span_range.0) * 1.0e-4).max(1.0e-4);
    fastener_positions_with_end_obstructions(span_range, end_obstructions, fasteners)?
        .into_iter()
        .map(|position| {
            let frame = wing_surface_frame(wing, position, chord_fraction, surface)?;
            let before = wing_surface_frame(
                wing,
                (position - span_step).max(span_range.0),
                chord_fraction,
                surface,
            )?;
            let after = wing_surface_frame(
                wing,
                (position + span_step).min(span_range.1),
                chord_fraction,
                surface,
            )?;
            let tangent = subtract(after.point, before.point);
            let axis = normalize_array(cross_array(outward_direction, tangent)).ok_or(
                WingError::InvalidSpec("cannot determine longitudinal split flange normal"),
            )?;
            let center = add_scaled(
                frame.point,
                outward_direction,
                attachment_offset + flange_width * 0.5,
            );
            let half_length = flange_thickness * 0.5 + fasteners.cutter_overtravel;
            Ok(LongitudinalSplitFlangeFastenerCutter {
                position,
                cutter: oriented_cylinder(
                    add_scaled(center, axis, -half_length),
                    add_scaled(center, axis, half_length),
                    fasteners.clearance_diameter,
                    fasteners.circular_segments,
                )?,
            })
        })
        .collect()
}

fn transverse_fastener_interfaces(
    wing: &WingSpec,
    seam_span: f64,
    inner_margin: f64,
    outer_margin: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<TransverseFastenerInterfaces, WingError> {
    let chord_fractions = fastener_chord_fractions(wing, seam_span, fasteners)?;
    let seam_normal = transverse_section_normal(wing, seam_span)?;
    let radial_offset = (inner_margin + outer_margin) * 0.5;
    let mut interfaces = Vec::with_capacity(chord_fractions.len() * 2);
    for surface in [WingSurface::Lower, WingSurface::Upper] {
        for &chord_fraction in &chord_fractions {
            let frame = wing_surface_frame(wing, seam_span, chord_fraction, surface)?;
            let radial_direction = normalize_array(add_scaled(
                frame.outward_normal,
                seam_normal,
                -dot(frame.outward_normal, seam_normal),
            ))
            .ok_or(WingError::InvalidSpec(
                "cannot project fastener placement into the seam plane",
            ))?;
            interfaces.push((
                surface,
                add_scaled(frame.point, radial_direction, radial_offset),
            ));
        }
    }
    Ok(TransverseFastenerInterfaces {
        seam_normal,
        placements: interfaces,
    })
}

fn fastener_chord_fractions(
    wing: &WingSpec,
    seam_span: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<f64>, WingError> {
    let chord = interpolate_station_extended(wing, seam_span)?.chord;
    Ok(fastener_positions((0.0, chord), fasteners)?
        .into_iter()
        .map(|position| position / chord)
        .collect())
}

fn fastener_positions(
    range: (f64, f64),
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<f64>, WingError> {
    fastener_positions_with_end_obstructions(range, FlangeEndObstructions::default(), fasteners)
}

fn fastener_positions_with_end_obstructions(
    range: (f64, f64),
    end_obstructions: FlangeEndObstructions,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<Vec<f64>, WingError> {
    if !range.0.is_finite()
        || !range.1.is_finite()
        || range.1 <= range.0
        || !end_obstructions.start.is_finite()
        || end_obstructions.start < 0.0
        || !end_obstructions.end.is_finite()
        || end_obstructions.end < 0.0
    {
        return Err(WingError::InvalidSpec(
            "flange range and end obstructions must be valid",
        ));
    }
    let head_radius = fasteners.head_diameter * 0.5;
    let standard_setback = fasteners.head_diameter * 2.0;
    let start_setback = standard_setback.max(end_obstructions.start + head_radius);
    let end_setback = standard_setback.max(end_obstructions.end + head_radius);

    match &fasteners.layout {
        WingFlangeFastenerLayout::MaximumSpacing(maximum_spacing) => {
            if !maximum_spacing.is_finite() || *maximum_spacing <= 0.0 {
                return Err(WingError::InvalidSpec(
                    "maximum flange fastener spacing must be finite and positive",
                ));
            }
            let positions = EndAnchoredDistribution {
                end_setback: 0.0,
                maximum_spacing: *maximum_spacing,
                minimum_positions: 2,
            }
            .positions((range.0 + start_setback, range.1 - end_setback))
            .map_err(|_| {
                WingError::InvalidSpec("flange cannot fit end-anchored fasteners at this spacing")
            })?;
            if positions
                .windows(2)
                .any(|pair| pair[1] - pair[0] < fasteners.head_diameter)
            {
                return Err(WingError::InvalidSpec(
                    "flange is too short to keep fastener heads separate",
                ));
            }
            Ok(positions)
        }
        WingFlangeFastenerLayout::Fractions(fractions) => {
            if fractions.is_empty()
                || fractions
                    .iter()
                    .any(|fraction| !fraction.is_finite() || *fraction <= 0.0 || *fraction >= 1.0)
                || fractions.windows(2).any(|pair| pair[1] <= pair[0])
            {
                return Err(WingError::InvalidSpec(
                    "flange fastener fractions must increase inside the flange",
                ));
            }
            let length = range.1 - range.0;
            let positions: Vec<f64> = fractions
                .iter()
                .map(|fraction| range.0 + length * fraction)
                .collect();
            let clear_start = range.0 + end_obstructions.start + head_radius;
            let clear_end = range.1 - end_obstructions.end - head_radius;
            if positions
                .iter()
                .any(|position| *position < clear_start || *position > clear_end)
            {
                return Err(WingError::InvalidSpec(
                    "explicit flange fastener position overlaps an end obstruction",
                ));
            }
            Ok(positions)
        }
    }
}

fn validate_through_flange(
    inner_margin: f64,
    outer_margin: f64,
    negative_depth: f64,
    positive_depth: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<(), WingError> {
    if ![
        inner_margin,
        outer_margin,
        negative_depth,
        positive_depth,
        fasteners.head_diameter,
        fasteners.clearance_diameter,
        fasteners.cutter_overtravel,
    ]
    .into_iter()
    .all(f64::is_finite)
        || inner_margin < 0.0
        || outer_margin <= inner_margin
        || outer_margin - inner_margin <= fasteners.head_diameter
        || negative_depth <= 0.0
        || positive_depth <= 0.0
        || fasteners.head_diameter <= fasteners.clearance_diameter
        || fasteners.clearance_diameter <= 0.0
        || fasteners.cutter_overtravel <= 0.0
        || fasteners.circular_segments < 8
    {
        return Err(WingError::InvalidSpec(
            "through-flange fasteners need safe flange and hardware dimensions",
        ));
    }
    Ok(())
}

fn validate(
    shell_thickness: f64,
    outer_margin: f64,
    ramp_length: f64,
    bed_thickness: f64,
    fasteners: &WingFlangeFastenerSpec,
) -> Result<(), WingError> {
    if ![
        shell_thickness,
        outer_margin,
        ramp_length,
        bed_thickness,
        fasteners.head_diameter,
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
        || fasteners.head_diameter <= fasteners.clearance_diameter
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
    if center_offset - fasteners.head_diameter * 0.5 <= shell_thickness
        || center_offset + fasteners.pilot_diameter * 0.5 >= pilot_end_margin
        || center_offset + fasteners.head_diameter * 0.5 >= outer_margin
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
    use crate::{
        WingBaseAttachmentSettings, chord_region_with_span_margins, preset,
        transverse_flange_blank, wing_base_attachment_geometry,
    };

    #[test]
    fn cutters_are_flange_centered_coaxial_and_stop_inside_the_ramp() {
        let wing = preset("tapered").unwrap();
        let fasteners = TransverseFlangeFastenerSpec::default();
        let cutters =
            transverse_flange_fastener_cutters(&wing, 200.0, 4.0, 15.2, 12.0, 3.0, &fasteners)
                .unwrap();
        let chord_fractions = fastener_chord_fractions(&wing, 200.0, &fasteners).unwrap();

        assert_eq!(cutters.len(), 6);
        for (index, cutters) in cutters.into_iter().enumerate() {
            let clearance = cutters.clearance.bounding_box();
            let pilot = cutters.pilot.bounding_box();
            let chord_fraction = chord_fractions[index % chord_fractions.len()];
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
    fn maximum_spacing_anchors_ends_and_scales_with_local_chord() {
        let fasteners = TransverseFlangeFastenerSpec::default();
        let fractions_for = |chord| {
            let mut wing = preset("rectangular").unwrap();
            for station in &mut wing.stations {
                station.chord = chord;
            }
            fastener_chord_fractions(&wing, 200.0, &fasteners)
                .unwrap()
                .into_iter()
                .map(|fraction| fraction * chord)
                .collect::<Vec<_>>()
        };

        let short = fractions_for(100.0);
        assert_eq!(short, vec![12.0, 88.0]);

        let long = fractions_for(500.0);
        assert_eq!(long.len(), 6);
        assert!((long[0] - 12.0).abs() < 1.0e-9);
        assert!((long[5] - 488.0).abs() < 1.0e-9);
        assert!(long.windows(2).all(|pair| pair[1] - pair[0] <= 100.0));
    }

    #[test]
    fn explicit_chord_fractions_remain_available() {
        let wing = preset("rectangular").unwrap();
        let fasteners = WingFlangeFastenerSpec {
            layout: WingFlangeFastenerLayout::Fractions(vec![0.2, 0.5, 0.8]),
            ..TransverseFlangeFastenerSpec::default()
        };

        assert_eq!(
            fastener_chord_fractions(&wing, 200.0, &fasteners).unwrap(),
            vec![0.2, 0.5, 0.8]
        );

        let reversed = WingFlangeFastenerSpec {
            layout: WingFlangeFastenerLayout::Fractions(vec![0.8, 0.2]),
            ..WingFlangeFastenerSpec::default()
        };
        assert!(fastener_chord_fractions(&wing, 200.0, &reversed).is_err());
    }

    #[test]
    fn explicit_fastener_position_cannot_overlap_a_sloped_end() {
        let fasteners = WingFlangeFastenerSpec {
            layout: WingFlangeFastenerLayout::Fractions(vec![0.25, 0.95]),
            ..WingFlangeFastenerSpec::default()
        };

        assert!(
            fastener_positions_with_end_obstructions(
                (0.0, 200.0),
                FlangeEndObstructions {
                    start: 0.0,
                    end: 12.0,
                },
                &fasteners,
            )
            .is_err()
        );
    }

    #[test]
    fn base_through_cutters_cross_both_mating_flange_bodies() {
        let wing = preset("tapered").unwrap();
        let geometry = wing_base_attachment_geometry(
            &wing,
            -4.0,
            WingBaseAttachmentSettings {
                flange_width: 15.2,
                axial_thickness: 3.0,
                registration: Default::default(),
            },
        )
        .unwrap();
        let cutters = transverse_through_flange_fastener_cutters(
            &wing,
            -4.0,
            4.0,
            15.2,
            3.0,
            3.0,
            &WingFlangeFastenerSpec::default(),
        )
        .unwrap();

        assert_eq!(cutters.len(), 6);
        for fastener in cutters {
            assert!(geometry.mold_flange.intersection(&fastener.cutter).volume() > 0.0);
            assert!(
                geometry
                    .sealing_profile
                    .intersection(&fastener.cutter)
                    .volume()
                    > 0.0
            );
        }
    }

    #[test]
    fn edge_through_cutters_cross_both_parting_flanges() {
        let wing = preset("rectangular").unwrap();
        let lower = chord_region_with_span_margins(&wing, -3.0, 0.0, 12.0, (0.0, 0.0)).unwrap();
        let upper = chord_region_with_span_margins(&wing, 0.0, 3.0, 12.0, (0.0, 0.0)).unwrap();
        let fasteners = WingFlangeFastenerSpec::default();

        for edge in [WingEdge::Leading, WingEdge::Trailing] {
            let cutters = longitudinal_edge_fastener_cutters(
                &wing,
                edge,
                (0.0, 200.0),
                12.0,
                3.0,
                FlangeEndObstructions::default(),
                &fasteners,
            )
            .unwrap();
            assert_eq!(
                cutters
                    .iter()
                    .map(|cutter| cutter.position)
                    .collect::<Vec<_>>(),
                vec![12.0, 100.0, 188.0]
            );
            for fastener in cutters {
                assert!(lower.intersection(&fastener.cutter).volume() > 0.0);
                assert!(upper.intersection(&fastener.cutter).volume() > 0.0);
            }
        }
    }

    #[test]
    fn edge_fastener_head_clears_a_sloped_end() {
        let wing = preset("rectangular").unwrap();
        let fasteners = WingFlangeFastenerSpec::default();
        let cutters = longitudinal_edge_fastener_cutters(
            &wing,
            WingEdge::Leading,
            (0.0, 200.0),
            12.0,
            3.0,
            FlangeEndObstructions {
                start: 0.0,
                end: 12.0,
            },
            &fasteners,
        )
        .unwrap();
        let positions: Vec<f64> = cutters.iter().map(|cutter| cutter.position).collect();

        assert_eq!(positions, vec![12.0, 98.5, 185.0]);
        assert_eq!(200.0 - positions[2], 12.0 + fasteners.head_diameter * 0.5);
    }

    #[test]
    fn longitudinal_split_cutters_cross_the_complete_flange_thickness() {
        let wing = preset("rectangular").unwrap();
        let cutters = longitudinal_split_flange_fastener_cutters(
            &wing,
            0.5,
            (0.0, 200.0),
            WingSurface::Upper,
            [0.0, 0.0, 1.0],
            3.2,
            12.0,
            3.0,
            FlangeEndObstructions::default(),
            &WingFlangeFastenerSpec::default(),
        )
        .unwrap();

        assert_eq!(cutters.len(), 3);
        for fastener in cutters {
            let bounds = fastener.cutter.bounding_box();
            assert!((bounds.max.x - bounds.min.x - 4.0).abs() < 1.0e-9);
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
