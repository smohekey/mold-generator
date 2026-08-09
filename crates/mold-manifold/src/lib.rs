//! Manifold-backed geometry implementation.
//!
//! This crate is the first concrete geometry backend for the mold generator.
//! The initial target is triangle-mesh workflows using STL for interchange.
//! STEP support is intentionally kept out of `mold-core`; a future B-rep or
//! STEP adapter can be added without changing the mold-generation algorithms.

pub use manifold_rust::manifold::Manifold;

/// Marker for the Manifold implementation of the geometry backend.
///
/// The `SolidKernel` implementation and STL conversion code will live here.
/// Keeping this type in its own crate prevents Manifold-specific mesh types
/// from leaking into `mold-core`.
#[derive(Debug, Default, Clone, Copy)]
pub struct ManifoldKernel;
