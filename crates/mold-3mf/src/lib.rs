use std::{
    fmt,
    fs::File,
    io::{self, Write},
    path::Path,
};

use mold_manifold::ManifoldSolid;
use zip::{CompressionMethod, ZipWriter, result::ZipError, write::SimpleFileOptions};

pub struct ThreeMfObject<'a> {
    pub name: String,
    pub solid: &'a ManifoldSolid,
}

#[derive(Debug)]
pub enum ThreeMfError {
    Io(io::Error),
    Zip(ZipError),
}

impl fmt::Display for ThreeMfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::Zip(error) => write!(f, "ZIP error: {error}"),
        }
    }
}

impl std::error::Error for ThreeMfError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Zip(error) => Some(error),
        }
    }
}

impl From<io::Error> for ThreeMfError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<ZipError> for ThreeMfError {
    fn from(value: ZipError) -> Self {
        Self::Zip(value)
    }
}

pub fn write_3mf(
    path: impl AsRef<Path>,
    title: &str,
    objects: &[ThreeMfObject<'_>],
) -> Result<(), ThreeMfError> {
    let file = File::create(path)?;
    let mut zip = ZipWriter::new(file);
    let options = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);

    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(CONTENT_TYPES.as_bytes())?;

    zip.start_file("_rels/.rels", options)?;
    zip.write_all(ROOT_RELS.as_bytes())?;

    zip.start_file("3D/3dmodel.model", options)?;
    write_model_xml(&mut zip, title, objects)?;

    zip.finish()?;
    Ok(())
}

fn write_model_xml(
    out: &mut impl Write,
    title: &str,
    objects: &[ThreeMfObject<'_>],
) -> io::Result<()> {
    write!(
        out,
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n  <metadata name=\"Title\">{}</metadata>\n  <resources>\n",
        escape_xml(title)
    )?;

    for (index, object) in objects.iter().enumerate() {
        let object_id = index + 1;
        let mesh = object.solid.0.as_original().get_mesh_gl64(-1);
        let stride = mesh.num_prop as usize;

        writeln!(
            out,
            "    <object id=\"{object_id}\" name=\"{}\" type=\"model\">",
            escape_xml(&object.name)
        )?;
        writeln!(out, "      <mesh>")?;
        writeln!(out, "        <vertices>")?;
        for vertex in mesh.vert_properties.chunks_exact(stride) {
            writeln!(
                out,
                "          <vertex x=\"{}\" y=\"{}\" z=\"{}\" />",
                vertex[0], vertex[1], vertex[2]
            )?;
        }
        writeln!(out, "        </vertices>")?;
        writeln!(out, "        <triangles>")?;
        for triangle in mesh.tri_verts.chunks_exact(3) {
            writeln!(
                out,
                "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\" />",
                triangle[0], triangle[1], triangle[2]
            )?;
        }
        writeln!(out, "        </triangles>")?;
        writeln!(out, "      </mesh>")?;
        writeln!(out, "    </object>")?;
    }

    writeln!(out, "  </resources>")?;
    writeln!(out, "  <build>")?;
    for index in 0..objects.len() {
        writeln!(out, "    <item objectid=\"{}\" />", index + 1)?;
    }
    writeln!(out, "  </build>")?;
    writeln!(out, "</model>")?;
    Ok(())
}

fn escape_xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

const CONTENT_TYPES: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />
</Types>
"#;

const ROOT_RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" />
</Relationships>
"#;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xml_escape_handles_names() {
        assert_eq!(escape_xml("upper & <left>"), "upper &amp; &lt;left&gt;");
    }
}
