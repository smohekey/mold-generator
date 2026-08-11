use crate::{
    WingError, WingSpec, WingSurface, cross_array, interpolate_station, normalize_array, subtract,
    surface_point,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WingSurfaceFrame {
    pub point: [f64; 3],
    pub outward_normal: [f64; 3],
}

/// Returns a point and outward normal on an analytic wing surface.
pub fn wing_surface_frame(
    wing: &WingSpec,
    span: f64,
    chord_fraction: f64,
    surface: WingSurface,
) -> Result<WingSurfaceFrame, WingError> {
    if !(0.0..=1.0).contains(&chord_fraction) {
        return Err(WingError::InvalidSpec(
            "surface frame needs a normalized chord fraction",
        ));
    }
    let first = wing
        .stations
        .first()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let last = wing
        .stations
        .last()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let span_step = ((last - first) * 1.0e-4).max(1.0e-4);
    let chord_step = 1.0e-4;
    let station = interpolate_station(wing, span)?;
    let point = surface_point(wing, &station, station.chord * chord_fraction, surface)?;

    let chord_before = (chord_fraction - chord_step).max(0.0);
    let chord_after = (chord_fraction + chord_step).min(1.0);
    let chord_tangent = subtract(
        surface_point(wing, &station, station.chord * chord_after, surface)?,
        surface_point(wing, &station, station.chord * chord_before, surface)?,
    );
    let span_before = (span - span_step).max(first);
    let span_after = (span + span_step).min(last);
    let before = interpolate_station(wing, span_before)?;
    let after = interpolate_station(wing, span_after)?;
    let span_tangent = subtract(
        surface_point(wing, &after, after.chord * chord_fraction, surface)?,
        surface_point(wing, &before, before.chord * chord_fraction, surface)?,
    );
    let mut outward_normal = normalize_array(cross_array(chord_tangent, span_tangent)).ok_or(
        WingError::InvalidSpec("cannot determine wing surface normal"),
    )?;
    if surface == WingSurface::Lower {
        outward_normal = outward_normal.map(|value| -value);
    }
    Ok(WingSurfaceFrame {
        point,
        outward_normal,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preset;

    #[test]
    fn upper_and_lower_frames_point_away_from_the_wing() {
        let wing = preset("twisted").unwrap();
        let upper = wing_surface_frame(&wing, 300.0, 0.5, WingSurface::Upper).unwrap();
        let lower = wing_surface_frame(&wing, 300.0, 0.5, WingSurface::Lower).unwrap();
        let length = |normal: [f64; 3]| {
            normal
                .into_iter()
                .map(|value| value * value)
                .sum::<f64>()
                .sqrt()
        };

        assert!((length(upper.outward_normal) - 1.0).abs() < 1.0e-9);
        assert!((length(lower.outward_normal) - 1.0).abs() < 1.0e-9);
        assert!(upper.outward_normal[2] > 0.0);
        assert!(lower.outward_normal[2] < 0.0);
    }
}
