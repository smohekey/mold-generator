use super::Naca4;
use crate::{WingError, WingSurface};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Airfoil {
    kind: AirfoilKind,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum AirfoilKind {
    Naca4(Naca4),
    Coordinates {
        name: &'static str,
        upper: &'static [[f64; 2]],
        lower: &'static [[f64; 2]],
    },
}

impl Airfoil {
    pub fn from_coordinates(
        name: &'static str,
        upper: &'static [[f64; 2]],
        lower: &'static [[f64; 2]],
    ) -> Result<Self, WingError> {
        let airfoil = Self::from_static_coordinates(name, upper, lower);
        airfoil.validate_coordinates()?;
        Ok(airfoil)
    }

    pub fn name(self) -> &'static str {
        match self.kind {
            AirfoilKind::Naca4(_) => "NACA 4-digit",
            AirfoilKind::Coordinates { name, .. } => name,
        }
    }

    pub(super) const fn from_static_coordinates(
        name: &'static str,
        upper: &'static [[f64; 2]],
        lower: &'static [[f64; 2]],
    ) -> Self {
        Self {
            kind: AirfoilKind::Coordinates { name, upper, lower },
        }
    }

    pub(crate) fn profile(self, point_count: usize, closed_trailing_edge: bool) -> Vec<(f64, f64)> {
        match self.kind {
            AirfoilKind::Naca4(naca) => naca.profile(point_count, closed_trailing_edge),
            AirfoilKind::Coordinates { upper, lower, .. } => {
                coordinate_profile(upper, lower, point_count)
            }
        }
    }

    pub(crate) fn surface_z(self, x: f64, surface: WingSurface, closed_trailing_edge: bool) -> f64 {
        match self.kind {
            AirfoilKind::Naca4(naca) => naca.surface_z(x, surface, closed_trailing_edge),
            AirfoilKind::Coordinates { upper, lower, .. } => match surface {
                WingSurface::Lower => interpolate_surface(lower, x),
                WingSurface::Upper => interpolate_surface(upper, x),
            },
        }
    }

    fn validate_coordinates(self) -> Result<(), WingError> {
        let AirfoilKind::Coordinates { name, upper, lower } = self.kind else {
            return Ok(());
        };
        let valid_surface = |points: &[[f64; 2]]| {
            points.len() >= 2
                && points
                    .iter()
                    .flatten()
                    .all(|coordinate| coordinate.is_finite())
                && points.windows(2).all(|pair| pair[1][0] > pair[0][0])
                && points[0][0].abs() < 1.0e-9
                && (points.last().unwrap()[0] - 1.0).abs() < 1.0e-9
        };
        if name.is_empty()
            || !valid_surface(upper)
            || !valid_surface(lower)
            || (upper[0][1] - lower[0][1]).abs() > 1.0e-9
            || (upper.last().unwrap()[1] - lower.last().unwrap()[1]).abs() > 1.0e-9
        {
            return Err(WingError::InvalidAirfoil(name.to_owned()));
        }
        Ok(())
    }
}

impl From<Naca4> for Airfoil {
    fn from(value: Naca4) -> Self {
        Self {
            kind: AirfoilKind::Naca4(value),
        }
    }
}

fn coordinate_profile(
    upper_points: &[[f64; 2]],
    lower_points: &[[f64; 2]],
    point_count: usize,
) -> Vec<(f64, f64)> {
    let mut upper = Vec::with_capacity(point_count);
    let mut lower = Vec::with_capacity(point_count);
    for index in 0..point_count {
        let beta = std::f64::consts::PI * index as f64 / (point_count - 1) as f64;
        let x = 0.5 * (1.0 - beta.cos());
        upper.push((x, interpolate_surface(upper_points, x)));
        lower.push((x, interpolate_surface(lower_points, x)));
    }

    let mut profile = Vec::with_capacity(2 * point_count - 2);
    profile.extend(upper.into_iter().rev());
    profile.extend(lower.into_iter().skip(1).take(point_count - 2));
    profile
}

fn interpolate_surface(points: &[[f64; 2]], x: f64) -> f64 {
    if x <= points[0][0] {
        return points[0][1];
    }
    if x >= points.last().unwrap()[0] {
        return points.last().unwrap()[1];
    }
    let end = points.partition_point(|point| point[0] < x);
    let start = end - 1;
    let fraction = (x - points[start][0]) / (points[end][0] - points[start][0]);
    points[start][1] + fraction * (points[end][1] - points[start][1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coordinate_airfoil_requires_normalized_matching_surfaces() {
        static UPPER: [[f64; 2]; 2] = [[0.0, 0.0], [1.0, 0.0]];
        static LOWER: [[f64; 2]; 2] = [[0.0, 0.0], [1.0, 0.0]];
        assert!(Airfoil::from_coordinates("flat", &UPPER, &LOWER).is_ok());

        static REVERSED: [[f64; 2]; 2] = [[1.0, 0.0], [0.0, 0.0]];
        assert!(Airfoil::from_coordinates("reversed", &REVERSED, &LOWER).is_err());
    }

    #[test]
    fn coordinate_profile_interpolates_each_surface() {
        static UPPER: [[f64; 2]; 3] = [[0.0, 0.0], [0.5, 0.1], [1.0, 0.0]];
        static LOWER: [[f64; 2]; 3] = [[0.0, 0.0], [0.5, -0.05], [1.0, 0.0]];
        let airfoil = Airfoil::from_coordinates("test", &UPPER, &LOWER).unwrap();

        assert!((airfoil.surface_z(0.25, WingSurface::Upper, true) - 0.05).abs() < 1.0e-12);
        assert!((airfoil.surface_z(0.25, WingSurface::Lower, true) + 0.025).abs() < 1.0e-12);
        assert_eq!(airfoil.profile(8, true).len(), 14);
    }
}
