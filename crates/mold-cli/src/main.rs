use std::{
    env,
    error::Error,
    io,
    num::NonZeroUsize,
    path::{Path, PathBuf},
    process::ExitCode,
};

use mold_core::{
    Axis, MoldSettings, SectionRegistration, generate_registered_sectioned_two_part_mold,
    generate_sectioned_two_part_mold,
};
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
    let split_axis = parse_axis(args.next().as_deref().unwrap_or("z"), "split axis")?;
    let section_count = match args.next() {
        Some(value) => value
            .parse::<NonZeroUsize>()
            .map_err(|_| invalid_input("section count must be a positive integer"))?,
        None => NonZeroUsize::MIN,
    };
    let section_axis = parse_axis(args.next().as_deref().unwrap_or("x"), "section axis")?;
    let registration = match args.next().as_deref().unwrap_or("none") {
        "none" => None,
        "default" => Some(SectionRegistration::default()),
        _ => return Err(invalid_input("registration must be none or default").into()),
    };

    if args.next().is_some() {
        return Err(invalid_input("too many arguments").into());
    }
    if !margin.is_finite() || margin <= 0.0 {
        return Err(invalid_input("margin must be finite and greater than zero").into());
    }

    let kernel = ManifoldKernel;
    let part = kernel.import_stl(&input)?;
    let settings = MoldSettings {
        margin: Vec3::new(margin, margin, margin),
    };
    let mold = match registration {
        Some(registration) => generate_registered_sectioned_two_part_mold(
            &kernel,
            &part,
            settings,
            split_axis,
            section_axis,
            section_count,
            registration,
        )?,
        None => generate_sectioned_two_part_mold(
            &kernel,
            &part,
            settings,
            split_axis,
            section_axis,
            section_count,
        )?,
    };

    export_sections(&kernel, &mold.negative, Path::new(&negative_output))?;
    export_sections(&kernel, &mold.positive, Path::new(&positive_output))?;

    Ok(())
}

fn export_sections(
    kernel: &ManifoldKernel,
    sections: &[mold_manifold::ManifoldSolid],
    output: &Path,
) -> Result<(), Box<dyn Error>> {
    for (index, section) in sections.iter().enumerate() {
        let path = section_output_path(output, index, sections.len());
        kernel.export_stl(section, path)?;
    }
    Ok(())
}

fn section_output_path(base: &Path, index: usize, count: usize) -> PathBuf {
    if count == 1 {
        return base.to_owned();
    }

    let stem = base
        .file_stem()
        .and_then(|value| value.to_str())
        .unwrap_or("mold");
    let extension = base.extension().and_then(|value| value.to_str());
    let digits = count.to_string().len().max(2);
    let file_name = match extension {
        Some(extension) => format!("{stem}-{:0digits$}.{extension}", index + 1),
        None => format!("{stem}-{:0digits$}", index + 1),
    };

    base.with_file_name(file_name)
}

fn parse_axis(value: &str, name: &str) -> Result<Axis, io::Error> {
    match value {
        "x" | "X" => Ok(Axis::X),
        "y" | "Y" => Ok(Axis::Y),
        "z" | "Z" => Ok(Axis::Z),
        _ => Err(invalid_input(&format!("{name} must be x, y, or z"))),
    }
}

fn invalid_input(message: &str) -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        format!(
            "{message}\nusage: mold-generator <input.stl> <negative.stl> <positive.stl> [margin] [split-axis] [sections] [section-axis] [registration]\n\n\
             margin defaults to 10.0 model units and is applied on every side\n\
             split-axis defaults to z and must be x, y, or z\n\
             sections defaults to 1 and must be a positive integer\n\
             section-axis defaults to x and must be x, y, or z\n\
             registration defaults to none and may be none or default\n\
             when sections > 1, output names are suffixed with -01, -02, etc."
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_section_preserves_requested_output_path() {
        assert_eq!(
            section_output_path(Path::new("left.stl"), 0, 1),
            PathBuf::from("left.stl")
        );
    }

    #[test]
    fn multiple_sections_get_numbered_output_paths() {
        assert_eq!(
            section_output_path(Path::new("out/left.stl"), 1, 12),
            PathBuf::from("out/left-02.stl")
        );
    }
}
