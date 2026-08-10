//! Reusable generation workflow for segmented, printable wing molds.

mod generator;

pub use generator::WingMoldGenerator;
pub use mold_wing_geometry::{Naca4, WingError, WingSpec, WingStation, preset};
