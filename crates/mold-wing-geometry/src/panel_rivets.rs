use manifold_rust::{
    linalg::Vec3,
    manifold::Manifold,
    types::{Error as ManifoldError, OpType},
};

use crate::{WingError, WingSpec, WingSurface, wing_surface_frame};

/// A normalized panel grid whose edges receive rows of spherical rivet heads.
///
/// Span and chord values are fractions of the wing span and local chord. The
/// physical dimensions use the same model units as the wing.
#[derive(Debug, Clone, PartialEq)]
pub struct WingPanelRivetSpec {
    pub surfaces: Vec<WingSurface>,
    pub span_edges: Vec<f64>,
    pub chord_edges: Vec<f64>,
    pub span_range: (f64, f64),
    pub chord_range: (f64, f64),
    pub spacing: f64,
    pub head_radius: f64,
    pub head_height: f64,
    pub path_samples: usize,
    pub circular_segments: i32,
}

impl WingPanelRivetSpec {
    pub fn scaled(mut self, factor: f64) -> Result<Self, WingError> {
        if !factor.is_finite() || factor <= 0.0 {
            return Err(WingError::InvalidSpec(
                "panel-rivet scale must be finite and positive",
            ));
        }
        self.spacing *= factor;
        self.head_radius *= factor;
        self.head_height *= factor;
        Ok(self)
    }

    fn validate(&self) -> Result<(), WingError> {
        let normalized_range = |range: (f64, f64)| {
            range.0.is_finite()
                && range.1.is_finite()
                && range.0 >= 0.0
                && range.1 <= 1.0
                && range.0 < range.1
        };
        let within = |value: &f64, range: (f64, f64)| {
            value.is_finite() && *value >= range.0 && *value <= range.1
        };
        if self.surfaces.is_empty()
            || !normalized_range(self.span_range)
            || !normalized_range(self.chord_range)
            || self
                .span_edges
                .iter()
                .any(|value| !within(value, self.span_range))
            || self
                .chord_edges
                .iter()
                .any(|value| !within(value, self.chord_range))
            || self.span_edges.is_empty() && self.chord_edges.is_empty()
            || ![self.spacing, self.head_radius, self.head_height]
                .into_iter()
                .all(f64::is_finite)
            || self.spacing <= 0.0
            || self.head_radius <= 0.0
            || self.head_height <= 0.0
            || self.head_height > self.head_radius
            || self.path_samples < 2
            || self.circular_segments < 8
        {
            return Err(WingError::InvalidSpec(
                "panel rivets need normalized edges, positive head dimensions, and path samples",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
struct SurfaceSample {
    point: [f64; 3],
    normal: [f64; 3],
}

/// Builds the partially embedded rivet-head solid described by a panel grid.
///
/// Union this solid with the source wing for visual output and subtract it
/// from the mold shell to transfer the same detail to the molding face.
pub fn panel_rivet_heads(
    wing: &WingSpec,
    rivets: &WingPanelRivetSpec,
) -> Result<Manifold, WingError> {
    rivets.validate()?;
    let first_span = wing
        .stations
        .first()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let last_span = wing
        .stations
        .last()
        .ok_or(WingError::InvalidSpec("wing has no stations"))?
        .span;
    let span_at = |fraction| first_span + (last_span - first_span) * fraction;
    let mut samples = Vec::new();

    for &surface in &rivets.surfaces {
        let mut surface_samples = Vec::new();
        for &span_fraction in &rivets.span_edges {
            let span = span_at(span_fraction);
            let path = sample_path(rivets.path_samples, |parameter| {
                sample_surface(
                    wing,
                    span,
                    lerp(rivets.chord_range.0, rivets.chord_range.1, parameter),
                    surface,
                )
            })?;
            surface_samples.extend(resample_path(&path, rivets.spacing));
        }
        for &chord_fraction in &rivets.chord_edges {
            let path = sample_path(rivets.path_samples, |parameter| {
                sample_surface(
                    wing,
                    span_at(lerp(rivets.span_range.0, rivets.span_range.1, parameter)),
                    chord_fraction,
                    surface,
                )
            })?;
            surface_samples.extend(resample_path(&path, rivets.spacing));
        }
        deduplicate(&mut surface_samples, rivets.head_radius * 1.5);
        samples.extend(surface_samples);
    }

    if samples.is_empty() {
        return Err(WingError::InvalidSpec(
            "panel-rivet paths are too short for the requested spacing",
        ));
    }
    let heads: Vec<Manifold> = samples
        .into_iter()
        .map(|sample| {
            let center = add_scaled(
                sample.point,
                sample.normal,
                rivets.head_height - rivets.head_radius,
            );
            Manifold::sphere(rivets.head_radius, rivets.circular_segments)
                .translate(Vec3::new(center[0], center[1], center[2]))
        })
        .collect();
    let heads = Manifold::batch_boolean(&heads, OpType::Add);
    if heads.status() != ManifoldError::NoError {
        return Err(WingError::Geometry(heads.status().to_string()));
    }
    Ok(heads)
}

fn sample_path(
    samples: usize,
    mut at: impl FnMut(f64) -> Result<SurfaceSample, WingError>,
) -> Result<Vec<SurfaceSample>, WingError> {
    (0..=samples)
        .map(|index| at(index as f64 / samples as f64))
        .collect()
}

fn sample_surface(
    wing: &WingSpec,
    span: f64,
    chord_fraction: f64,
    surface: WingSurface,
) -> Result<SurfaceSample, WingError> {
    let frame = wing_surface_frame(wing, span, chord_fraction, surface)?;
    Ok(SurfaceSample {
        point: frame.point,
        normal: frame.outward_normal,
    })
}

fn resample_path(path: &[SurfaceSample], spacing: f64) -> Vec<SurfaceSample> {
    let mut cumulative = Vec::with_capacity(path.len());
    cumulative.push(0.0);
    for pair in path.windows(2) {
        cumulative
            .push(cumulative.last().copied().unwrap() + distance(pair[0].point, pair[1].point));
    }
    let total = cumulative.last().copied().unwrap_or(0.0);
    let count = (total / spacing).floor() as usize;
    if count == 0 {
        return Vec::new();
    }
    let step = total / (count + 1) as f64;
    (1..=count)
        .map(|index| interpolate_path(path, &cumulative, step * index as f64))
        .collect()
}

fn interpolate_path(path: &[SurfaceSample], cumulative: &[f64], distance: f64) -> SurfaceSample {
    let index = cumulative.partition_point(|value| *value < distance);
    let after = index.min(path.len() - 1);
    let before = after.saturating_sub(1);
    let segment = cumulative[after] - cumulative[before];
    let parameter = if segment > 0.0 {
        (distance - cumulative[before]) / segment
    } else {
        0.0
    };
    SurfaceSample {
        point: lerp3(path[before].point, path[after].point, parameter),
        normal: normalize(lerp3(path[before].normal, path[after].normal, parameter))
            .unwrap_or(path[before].normal),
    }
}

fn deduplicate(samples: &mut Vec<SurfaceSample>, minimum_distance: f64) {
    let minimum_distance_squared = minimum_distance * minimum_distance;
    let mut unique: Vec<SurfaceSample> = Vec::with_capacity(samples.len());
    for sample in samples.drain(..) {
        if unique.iter().all(|existing| {
            distance_squared(existing.point, sample.point) >= minimum_distance_squared
        }) {
            unique.push(sample);
        }
    }
    *samples = unique;
}

fn add_scaled(point: [f64; 3], direction: [f64; 3], distance: f64) -> [f64; 3] {
    [
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
        point[2] + direction[2] * distance,
    ]
}

fn normalize(value: [f64; 3]) -> Option<[f64; 3]> {
    let length = value[0].hypot(value[1]).hypot(value[2]);
    (length > 0.0).then(|| value.map(|component| component / length))
}

fn lerp(a: f64, b: f64, parameter: f64) -> f64 {
    a + (b - a) * parameter
}

fn lerp3(a: [f64; 3], b: [f64; 3], parameter: f64) -> [f64; 3] {
    [
        lerp(a[0], b[0], parameter),
        lerp(a[1], b[1], parameter),
        lerp(a[2], b[2], parameter),
    ]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    distance_squared(a, b).sqrt()
}

fn distance_squared(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{generate, preset};

    fn detail() -> WingPanelRivetSpec {
        WingPanelRivetSpec {
            surfaces: vec![WingSurface::Upper, WingSurface::Lower],
            span_edges: vec![0.25, 0.5, 0.75],
            chord_edges: vec![0.3, 0.65],
            span_range: (0.1, 0.9),
            chord_range: (0.1, 0.85),
            spacing: 30.0,
            head_radius: 1.0,
            head_height: 0.4,
            path_samples: 24,
            circular_segments: 8,
        }
    }

    #[test]
    fn panel_rivet_heads_intersect_and_project_from_both_wing_surfaces() {
        let wing = preset("elliptical").unwrap();
        let base = generate(&wing).unwrap();
        let heads = panel_rivet_heads(&wing, &detail()).unwrap();

        assert!(!heads.is_empty());
        assert!(heads.intersection(&base).volume() > 0.0);
        assert!(heads.difference(&base).volume() > 0.0);
    }

    #[test]
    fn panel_rivet_spec_rejects_edges_outside_the_panel_range() {
        let wing = preset("elliptical").unwrap();
        let mut detail = detail();
        detail.span_edges.push(0.95);

        assert!(panel_rivet_heads(&wing, &detail).is_err());
    }
}
