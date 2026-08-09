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

The CLI imports a closed manifold STL, builds a rectangular blank around its bounds, splits the blank through the source part, subtracts the source geometry from each half, and exports the two resulting mold parts:

```sh
cargo run -p mold-cli -- input.stl negative.stl positive.stl [margin] [axis]
```

`margin` defaults to `10.0` model units and is applied on every side. `axis` defaults to `z` and can be `x`, `y`, or `z`. The split plane currently passes through the midpoint of the source part along that axis.

This is the first printable two-part mold shape. It does not yet include registration features, structural ribs, spar exclusions, injection paths, venting, or longitudinal subdivision into printer-sized sections. Those features can be layered on in `mold-core` without coupling the algorithms to Fusion or a particular geometry kernel.
