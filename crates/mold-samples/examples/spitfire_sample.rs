use mold_wing::{WingMoldGenerator, WingPanelRivetSpec, WingSurface, preset};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    WingMoldGenerator::new(
        preset("elliptical")?,
        "target/sample-mold/spitfire",
        "spitfire-style-wing",
        "Elliptical warbird wing mold with panel-edge rivets",
    )
    .with_profile_points(64)
    .with_panel_rivets(WingPanelRivetSpec {
        surfaces: vec![WingSurface::Upper, WingSurface::Lower],
        span_edges: vec![0.16, 0.3, 0.45, 0.6, 0.74, 0.86],
        chord_edges: vec![0.22, 0.5, 0.76],
        span_range: (0.08, 0.92),
        chord_range: (0.08, 0.84),
        spacing: 20.0,
        head_radius: 0.9,
        head_height: 0.35,
        path_samples: 64,
        circular_segments: 12,
    })
    .generate()
}
