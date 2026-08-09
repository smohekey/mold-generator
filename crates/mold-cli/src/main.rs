use std::{env, error::Error, io, process::ExitCode};

use mold_core::{Axis, MoldSettings, generate_two_part_mold};
use mold_geometry::Vec3;
use mold_manifold::ManifoldKernel;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let input = args
        .next()
        .ok_or_else(|| invalid_input("missing input STL"))?;
    let negative_output = args
        .next()
        .ok_or_else(|| invalid_input("missing negative-side output STL"))?;
    let positive_output = args
        .next()
        .ok_or_else(|| invalid_input("missing positive-side output STL"))?;
    let margin = match args.next() {
        Some(value) => value
            .parse::<f64>()
            .map_err(|_| invalid_input("margin must be a number"))?,
        None => 10.0,
    };
    let split_axis = match args.next().as_deref() {
        None | Some("z") | Some("Z") => Axis::Z,
        Some("x") | Some("X") => Axis::X,
        Some("y") | Some("Y") => Axis::Y,
        Some(_) => return Err(invalid_input("split axis must be x, y, or z").into()),
    };

    if args.next().is_some() {
        return Err(invalid_input("too many arguments").into());
    }
    if !margin.is_finite() || margin <= 0.0 {
        return Err(invalid_input("margin must be finite and greater than zero").into());
    }

    let kernel = ManifoldKernel;
    let part = kernel.import_stl(&input)?;
    let mold = generate_two_part_mold(
        &kernel,
        &part,
        MoldSettings {
            margin: Vec3::new(margin, margin, margin),
        },
        split_axis,
    )?;
    kernel.export_stl(&mold.negative, &negative_output)?;
    kernel.export_stl(&mold.positive, &positive_output)?;

    Ok(())
}

fn invalid_input(message: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        format!(
            "{message}\nusage: mold-generator <input.stl> <negative.stl> <positive.stl> [margin] [axis]\n\n\
             margin defaults to 10.0 model units and is applied on every side\n\
             axis defaults to z and must be x, y, or z"
        ),
    )
}
