use mold_wing::{WingMoldGenerator, preset};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    WingMoldGenerator::new(
        preset("tapered")?,
        "target/sample-mold/straight",
        "straight-wing",
        "Straight tapered wing mold validation assembly",
    )
    .generate()
}
