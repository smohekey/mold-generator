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

The CLI currently imports a closed manifold STL, builds a rectangular blank around its bounds, subtracts the source part, and exports the resulting body as STL:

```sh
cargo run -p mold-cli -- input.stl output.stl [margin]
```

`margin` defaults to `10.0` model units and is applied on every side.

The resulting body is intentionally still a single closed block containing the cavity. It is a proof of the import → kernel → mold-core → export pipeline, not yet a printable mold. The next mold-generation step is to split that body into printable mold parts, after which registration features, structural ribs, spar exclusions, injection paths, and venting can be layered on without coupling those operations to Fusion or a particular geometry kernel.
