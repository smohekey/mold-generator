use super::Airfoil;

// UIUC Airfoil Data Site, goe601.dat, normalized to a shared [0, 0] leading edge:
// https://m-selig.ae.illinois.edu/ads/coord_seligFmt/goe601.dat
const UPPER: [[f64; 2]; 17] = [
    [0.0, 0.0],
    [0.01032, 0.02203],
    [0.02191, 0.03119],
    [0.04542, 0.04604],
    [0.06923, 0.05792],
    [0.09324, 0.06782],
    [0.14180, 0.08218],
    [0.19091, 0.09109],
    [0.29007, 0.09950],
    [0.38977, 0.10247],
    [0.49011, 0.09901],
    [0.59110, 0.08911],
    [0.69273, 0.07277],
    [0.79486, 0.05148],
    [0.89718, 0.02822],
    [0.94862, 0.01386],
    [1.0, 0.0],
];

const LOWER: [[f64; 2]; 17] = [
    [0.0, 0.0],
    [0.01463, -0.02104],
    [0.02785, -0.02822],
    [0.05384, -0.03812],
    [0.07948, -0.04455],
    [0.10487, -0.04851],
    [0.15557, -0.05544],
    [0.20596, -0.05940],
    [0.30606, -0.06039],
    [0.40571, -0.05693],
    [0.50486, -0.04851],
    [0.60397, -0.03960],
    [0.70298, -0.02970],
    [0.80199, -0.01980],
    [0.90099, -0.00990],
    [0.95050, -0.00495],
    [1.0, 0.0],
];

pub const GOE_601: Airfoil = Airfoil::from_static_coordinates("Göttingen 601", &UPPER, &LOWER);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WingSurface;

    #[test]
    fn goe_601_preserves_the_tabulated_peak_surfaces() {
        assert_eq!(
            Airfoil::from_coordinates("Göttingen 601", &UPPER, &LOWER).unwrap(),
            GOE_601
        );
        assert_eq!(GOE_601.name(), "Göttingen 601");
        assert!((GOE_601.surface_z(0.38977, WingSurface::Upper, true) - 0.10247).abs() < 1.0e-12);
        assert!((GOE_601.surface_z(0.30606, WingSurface::Lower, true) + 0.06039).abs() < 1.0e-12);
    }
}
