use mold_wing::{WingMoldGenerator, preset};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    WingMoldGenerator::new(
        preset("gull")?,
        "target/sample-mold",
        "gull-wing",
        "Gull wing mold validation assembly",
    )
    .with_model_scale(1.37)
    .generate()
}
