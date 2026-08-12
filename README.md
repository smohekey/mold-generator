# mold-generator

A generic mold generator for STL/STEP-derived geometry, with a Rust core that is independent of any particular CAD application.

## Architecture

- `mold-geometry` defines backend-neutral geometry and the `SolidKernel` abstraction used by mold algorithms.
- `mold-manifold` is the first concrete backend, using the pure-Rust `manifold-rust` crate for triangle-mesh CSG and STL conversion.
- `mold-core` contains mold-generation logic and depends only on `mold-geometry`.
- `mold-wing-geometry` defines deterministic wing specifications and geometry independently of mold-generation policy.
- `mold-wing` contains the reusable wing-mold generation workflow.
- `mold-cli` is the command-line frontend.
- `mold-samples` contains configuration-only example binaries.
- CAD integrations such as Autodesk Fusion should remain thin adapters around the core.

## File formats

The initial implementation targets STL import and export. STEP is a future requirement, but it is deliberately not part of the core API. There are two intended upgrade paths:

1. Import STEP through a future adapter, tessellate it, and pass the resulting mesh into the Manifold backend. This provides STEP input while preserving the STL/mesh processing pipeline.
2. Add a future B-rep `SolidKernel` backend for workflows that require native CAD topology and proper STEP export.

This separation allows the mold-generation algorithms to remain unchanged as richer CAD formats are added.

## Wing geometry and molds

`mold-wing-geometry` generates deterministic closed wing solids from NACA 4-digit airfoils. Coordinates use X for chord, Y for span, and Z for vertical displacement. The generator lofts explicit spanwise stations, so sweep, taper, dihedral, twist, and non-linear gull-wing geometry can be exercised without relying on external STL fixtures.

Available presets are `rectangular`, `tapered`, `swept`, `dihedral`, `twisted`, and `gull`:

```sh
cargo run -p mold-wing-geometry -- gull gull-wing.stl
```

The geometry API exposes `WingSpec`, `WingStation`, and `Naca4`, allowing consumers and tests to construct custom deterministic wings programmatically. It intentionally does not depend on `mold-core`, so mold-generation tests do not generate their input geometry with the algorithms under test.

`mold-wing` owns the segmentation, flange, registration, tiling, validation, and export workflow. The gull and straight tapered samples contain only the wing preset and artifact configuration:

```sh
cargo run -p mold-samples --example gull_sample
cargo run -p mold-samples --example straight_sample
cargo run -p mold-samples --example spitfire_sample
```

Wing molds use a minimum 4 mm shell without external support ribs. The shell grows away from the wing and the original wing solid remains the cavity cutter, so increasing wall thickness does not change modeled surface detail on the molding face. For configured protruding details, their height is added to the smooth outer offset so at least 4 mm remains behind the resulting cavity. Detail fidelity is instead bounded by the source mesh, Boolean operations, and the target printer and material.

The Spitfire-style sample uses the reusable elliptical wing preset and a normalized panel grid with rivet rows on both wing surfaces. Panel-edge sampling and rivet geometry live in the production wing crates; the sample supplies only shape, detail, and artifact configuration. CI generates and uploads a separate STL/3MF artifact for every sample.

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
