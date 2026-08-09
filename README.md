# mold-generator

A generic mold generator for STL/STEP-derived geometry, with a Rust core that is independent of any particular CAD application.

## Architecture

- `mold-geometry` defines the geometry/kernel abstraction used by the mold algorithms.
- `mold-core` contains mold-generation logic and depends only on that abstraction.
- `mold-cli` is the command-line frontend and will host import/export plumbing.
- CAD integrations such as Autodesk Fusion should remain thin adapters around the core.

The first milestone is a mesh-backed implementation capable of creating a mold blank and subtracting the source part robustly. From there the core can grow splitting, ribs, registration features, spar exclusions, injection paths, and venting without coupling those operations to Fusion.
