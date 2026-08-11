use crate::{WingError, WingSurface};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Naca4 {
    pub max_camber: f64,
    pub camber_position: f64,
    pub thickness: f64,
}

impl Naca4 {
    pub fn parse(code: &str) -> Result<Self, WingError> {
        if code.len() != 4 || !code.bytes().all(|byte| byte.is_ascii_digit()) {
            return Err(WingError::InvalidAirfoil(code.to_owned()));
        }
        let digits: Vec<u32> = code
            .chars()
            .map(|character| character.to_digit(10).unwrap())
            .collect();
        Ok(Self {
            max_camber: digits[0] as f64 / 100.0,
            camber_position: digits[1] as f64 / 10.0,
            thickness: (digits[2] * 10 + digits[3]) as f64 / 100.0,
        })
    }

    pub(super) fn profile(self, point_count: usize, closed_trailing_edge: bool) -> Vec<(f64, f64)> {
        let mut upper = Vec::with_capacity(point_count);
        let mut lower = Vec::with_capacity(point_count);
        for index in 0..point_count {
            let beta = std::f64::consts::PI * index as f64 / (point_count - 1) as f64;
            let x = 0.5 * (1.0 - beta.cos());
            let trailing = if closed_trailing_edge {
                -0.1036
            } else {
                -0.1015
            };
            let thickness = 5.0
                * self.thickness
                * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x
                    + 0.2843 * x * x * x
                    + trailing * x * x * x * x);
            let (camber, slope) = self.camber(x);
            let theta = slope.atan();
            upper.push((
                x - thickness * theta.sin(),
                camber + thickness * theta.cos(),
            ));
            lower.push((
                x + thickness * theta.sin(),
                camber - thickness * theta.cos(),
            ));
        }

        let mut profile = Vec::with_capacity(2 * point_count - 2);
        profile.extend(upper.into_iter().rev());
        profile.extend(lower.into_iter().skip(1).take(point_count - 2));
        profile
    }

    pub(super) fn surface_z(self, x: f64, surface: WingSurface, closed_trailing_edge: bool) -> f64 {
        let trailing = if closed_trailing_edge {
            -0.1036
        } else {
            -0.1015
        };
        let thickness = 5.0
            * self.thickness
            * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x.powi(2)
                + 0.2843 * x.powi(3)
                + trailing * x.powi(4));
        let (camber, _) = self.camber(x);
        match surface {
            WingSurface::Lower => camber - thickness,
            WingSurface::Upper => camber + thickness,
        }
    }

    fn camber(self, x: f64) -> (f64, f64) {
        let maximum = self.max_camber;
        let position = self.camber_position;
        if maximum == 0.0 || position == 0.0 {
            return (0.0, 0.0);
        }
        if x < position {
            (
                maximum / (position * position) * (2.0 * position * x - x * x),
                2.0 * maximum / (position * position) * (position - x),
            )
        } else {
            (
                maximum / ((1.0 - position) * (1.0 - position))
                    * ((1.0 - 2.0 * position) + 2.0 * position * x - x * x),
                2.0 * maximum / ((1.0 - position) * (1.0 - position)) * (position - x),
            )
        }
    }
}
