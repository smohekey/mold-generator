use mold_samples::WingMoldSample;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    WingMoldSample::new(
        mold_test_models::preset("tapered")?,
        "target/sample-mold/straight",
        "straight-wing",
        "Straight tapered wing mold validation assembly",
    )
    .generate()
}
