# mold-generator

A generic mold generator for STL/STEP-derived geometry, with a Rust core that is independent of any particular CAD application.

## Architecture

- `mold-geometry` defines backend-neutral geometry and the `SolidKernel` abstraction used by mold algorithms.
- `mold-manifold` is the first concrete backend, using the pure-Rust `manifold-rust` crate for triangle-mesh CSG and STL conversion.
- `mold-core` contains mold-generation logic and depends only on `mold-geometry`.
- `mold-cli` is the command-line frontend.
- CAD integrations such as Autodesk Fusion should remain thin adapters around the core.

## File formats

The initial implementation targets STL import and export. STEP is a future requirement, but it is deliberately not part of the core API. There are two intended upgrade paths:

1. Import STEP through a future adapter, tessellate it, and pass the resulting mesh into the Manifold backend. This provides STEP input while preserving the STL/mesh processing pipeline.
2. Add a future B-rep `SolidKernel` backend for workflows that require native CAD topology and proper STEP export.

This separation allows the mold-generation algorithms to remain unchanged as richer CAD formats are added.

## Current vertical slice

The CLI imports a closed manifold STL, builds a rectangular blank around its bounds, splits the blank through the source part, optionally subdivides each half into printable sections, subtracts the source geometry, and exports the resulting mold pieces:

```sh
cargo run -p mold-cli -- \
    input.stl \
    negative.stl \
    positive.stl \
    [margin] \
    [split-axis] \
    [sections] \
    [section-axis]
```

Defaults are:

- `margin = 10.0` model units on every side
- `split-axis = z`
- `sections = 1`
- `section-axis = x`

For example, this creates a Z-split mold divided into four sections along X:

```sh
cargo run -p mold-cli -- wing.stl lower.stl upper.stl 10 z 4 x
```

The output files are then numbered `lower-01.stl` through `lower-04.stl` and `upper-01.stl` through `upper-04.stl`. With a single section, the exact requested output filenames are retained.

The split plane currently passes through the midpoint of the source part along the split axis. Sections are equal-width divisions of the complete padded mold blank along the section axis. When the split and section axes differ, this gives exactly `2 × N` mold pieces.

The next mold-generation work is to add geometry at the section and split interfaces: registration features, structural ribs/flanges, and then spar exclusions, injection paths, and venting. These remain operations in `mold-core` rather than Manifold-specific code.
