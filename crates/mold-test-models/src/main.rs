use std::{env, error::Error, io, process::ExitCode};

use mold_test_models::{generate, preset, write_stl};

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
    let preset_name = args
        .next()
        .ok_or_else(|| invalid_input("missing preset name"))?;
    let output = args
        .next()
        .ok_or_else(|| invalid_input("missing output STL"))?;
    if args.next().is_some() {
        return Err(invalid_input("too many arguments").into());
    }

    let spec = preset(&preset_name)?;
    let solid = generate(&spec)?;
    write_stl(&solid, output)?;
    Ok(())
}

fn invalid_input(message: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        format!(
            "{message}\nusage: mold-test-models <preset> <output.stl>\n\n\
             presets: rectangular, tapered, swept, dihedral, twisted, gull"
        ),
    )
}
