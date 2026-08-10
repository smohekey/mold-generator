# mold-generator

A generic mold generator for STL/STEP-derived geometry, with a Rust core that is independent of any particular CAD application.

## Architecture

- `mold-geometry` defines backend-neutral geometry and the `SolidKernel` abstraction used by mold algorithms.
- `mold-manifold` is the first concrete backend, using the pure-Rust `manifold-rust` crate for triangle-mesh CSG and STL conversion.
- `mold-core` contains mold-generation logic and depends only on `mold-geometry`.
- `mold-cli` is the command-line frontend.
- `mold-test-models` generates deterministic wing fixtures independently of `mold-core`.
- `mold-samples` owns the reusable wing-mold sample workflow; individual examples only select a
  wing specification and artifact metadata.
- CAD integrations such as Autodesk Fusion should remain thin adapters around the core.

## File formats

The initial implementation targets STL import and export. STEP is a future requirement, but it is deliberately not part of the core API. There are two intended upgrade paths:

1. Import STEP through a future adapter, tessellate it, and pass the resulting mesh into the Manifold backend. This provides STEP input while preserving the STL/mesh processing pipeline.
2. Add a future B-rep `SolidKernel` backend for workflows that require native CAD topology and proper STEP export.

This separation allows the mold-generation algorithms to remain unchanged as richer CAD formats are added.

## Wing test models

`mold-test-models` generates deterministic closed wing solids from NACA 4-digit airfoils. Coordinates use X for chord, Y for span, and Z for vertical displacement. The generator lofts explicit spanwise stations, so sweep, taper, dihedral, twist, and non-linear gull-wing geometry can be exercised without relying on external STL fixtures.

Available presets are `rectangular`, `tapered`, `swept`, `dihedral`, `twisted`, and `gull`:

```sh
cargo run -p mold-test-models -- gull gull-wing.stl
```

The library API also exposes `WingSpec`, `WingStation`, and `Naca4`, allowing tests to construct custom deterministic fixtures programmatically. The fixture generator intentionally does not depend on `mold-core`, so mold-generation tests do not generate their input geometry with the algorithms under test.

The gull and straight tapered samples exercise the same mold-generation workflow without
duplicating segmentation, flange, registration, webbing, tiling, validation, or export logic:

```sh
cargo run -p mold-samples --example gull_sample
cargo run -p mold-samples --example straight_sample
```

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
    [section-axis] \
    [registration]
```

Defaults are:

- `margin = 10.0` model units on every side
- `split-axis = z`
- `sections = 1`
- `section-axis = x`
- `registration = none`

For example, this creates a Z-split mold divided into four sections along X with mating registration keys:

```sh
cargo run -p mold-cli -- wing.stl lower.stl upper.stl 10 z 4 x default
```

The output files are then numbered `lower-01.stl` through `lower-04.stl` and `upper-01.stl` through `upper-04.stl`. With a single section, the exact requested output filenames are retained.

The split plane currently passes through the midpoint of the source part along the split axis. Sections are equal-width divisions of the complete padded mold blank along the section axis. When the split and section axes differ, this gives exactly `2 × N` mold pieces.

With `registration = default`, every internal section interface receives two rectangular male keys on the lower-index section and clearance sockets on the higher-index section. The default geometry uses a 4 mm key depth, 10 mm width, 4 mm height, 0.2 mm socket clearance, and a 2 mm inset from the outer mold face. The keys are placed in the outer-margin band on each mold half, away from the source-part cavity.

The current mold representation is still a solid rectangular block minus the source cavity. Structural support ribs would therefore be redundant at this stage; they become useful once the mold is represented as a thin shell. The next mold-generation work should focus on split-half registration and then the thin-shell/support-rib representation before spar exclusions, injection paths, and venting.
