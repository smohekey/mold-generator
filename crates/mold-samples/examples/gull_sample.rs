use mold_samples::WingMoldSample;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    WingMoldSample::new(
        mold_test_models::preset("gull")?,
        "target/sample-mold",
        "gull-wing",
        "Gull wing mold validation assembly",
    )
    .with_model_scale(1.2)
    .generate()
}
