# mold-generator

A generic mold generator for STL/STEP-derived geometry, with a Rust core that is independent of any particular CAD application.

## Architecture

- `mold-geometry` defines backend-neutral geometry and the `SolidKernel` abstraction used by mold algorithms.
- `mold-manifold` is the first concrete backend, using the pure-Rust `manifold-rust` crate for robust triangle-mesh CSG.
- `mold-core` contains mold-generation logic and depends only on `mold-geometry`.
- `mold-cli` is the command-line frontend and will host import/export plumbing.
- CAD integrations such as Autodesk Fusion should remain thin adapters around the core.

## File formats

The initial implementation targets STL import and export. `stl_io` handles STL serialization, while `mold-manifold` converts between STL triangle meshes and Manifold solids.

STEP is a future requirement, but it is deliberately not part of the core API. There are two intended upgrade paths:

1. Import STEP through a future adapter, tessellate it, and pass the resulting mesh into the Manifold backend. This provides STEP input while preserving the STL/mesh processing pipeline.
2. Add a future B-rep `SolidKernel` backend for workflows that require native CAD topology and proper STEP export.

This separation allows the mold-generation algorithms to remain unchanged as richer CAD formats are added.

## Initial milestone

The first milestone is an end-to-end STL workflow capable of importing a manifold mesh, creating a mold blank around it, subtracting the source part robustly, and exporting the result as STL. From there the core can grow splitting, ribs, registration features, spar exclusions, injection paths, and venting without coupling those operations to Fusion or to a particular geometry kernel.
