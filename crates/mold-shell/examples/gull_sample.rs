use std::{fs, num::NonZeroUsize, path::Path};

use manifold_rust::{manifold::Manifold, types::MeshGL64};
use mold_3mf::{ThreeMfObject, write_3mf};
use mold_core::Axis;
use mold_manifold::{ManifoldKernel, ManifoldSolid};
use mold_shell::{PartingRegions, ShellSettings, generate_sectioned_shell_mold_with_parting};
use mold_test_models::{WingSpec, WingStation};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = Path::new("target/sample-mold");
    fs::create_dir_all(output)?;
    let mut spec = mold_test_models::preset("gull")?; spec.profile_points = 24;
    let wing = mold_test_models::generate(&spec)?;
    mold_test_models::write_stl(&wing, output.join("gull-wing.stl"))?;
    let lower_region = chord_region(&spec,-500.0,0.0,80.0)?;
    let upper_region = chord_region(&spec,0.0,500.0,80.0)?;
    let lower_flange = chord_region(&spec,-3.0,0.0,12.0)?;
    let upper_flange = chord_region(&spec,0.0,3.0,12.0)?;

    // Asymmetric, FDM-friendly diamond keys. They are positioned in local
    // chord coordinates, so their insertion axis follows the local parting
    // normal even on the gull and twisted panels.
    let key_a = diamond_key(&spec, 0.32, -7.0, 7.0, 3.0, 0.0)?;
    let key_b = diamond_key(&spec, 0.72, -5.5, 5.5, 3.5, 0.0)?;
    let socket_a = diamond_key(&spec, 0.32, -7.3, 7.3, 3.25, 0.0)?;
    let socket_b = diamond_key(&spec, 0.72, -5.8, 5.8, 3.75, 0.0)?;

    let kernel=ManifoldKernel; let part=ManifoldSolid(wing);
    let lower_region=ManifoldSolid(lower_region);let upper_region=ManifoldSolid(upper_region);
    let lower_flange=ManifoldSolid(lower_flange);let upper_flange=ManifoldSolid(upper_flange);
    let key_a=ManifoldSolid(key_a);let key_b=ManifoldSolid(key_b);let socket_a=ManifoldSolid(socket_a);let socket_b=ManifoldSolid(socket_b);
    let keys=[&key_a,&key_b]; let sockets=[&socket_a,&socket_b];
    let mold=generate_sectioned_shell_mold_with_parting(&kernel,&part,PartingRegions{negative:&lower_region,positive:&upper_region,negative_flange:Some(&lower_flange),positive_flange:Some(&upper_flange),negative_keys:&keys,positive_sockets:&sockets},Axis::Y,NonZeroUsize::new(2).unwrap(),ShellSettings::default())?;
    for(index,piece)in mold.negative.iter().enumerate(){kernel.export_stl(piece,output.join(format!("mold-lower-{:02}.stl",index+1)))?;}
    for(index,piece)in mold.positive.iter().enumerate(){kernel.export_stl(piece,output.join(format!("mold-upper-{:02}.stl",index+1)))?;}
    let mut assembly=Vec::with_capacity(1+mold.negative.len()+mold.positive.len());assembly.push(ThreeMfObject{name:"wing".to_owned(),solid:&part});
    for(index,piece)in mold.negative.iter().enumerate(){assembly.push(ThreeMfObject{name:format!("mold-lower-{:02}",index+1),solid:piece});}
    for(index,piece)in mold.positive.iter().enumerate(){assembly.push(ThreeMfObject{name:format!("mold-upper-{:02}",index+1),solid:piece});}
    write_3mf(output.join("gull-wing-mold-assembly.3mf"),"Gull wing mold validation assembly",&assembly)?;
    println!("generated visual sample in {}",output.display());Ok(())
}

/// Generate two spanwise lozenges extruded across the local parting plane.
/// `chord_fraction` locates the feature relative to each station chord;
/// `half_width` is the diamond half-width along chord, `height` is insertion
/// depth, and `base_z` is normally zero (the chord plane).
fn diamond_key(spec:&WingSpec,chord_fraction:f64,span_min:f64,span_max:f64,height:f64,base_z:f64)->Result<Manifold,Box<dyn std::error::Error>>{
    let stations:Vec<_>=spec.stations.iter().filter(|s|s.span>=span_min&&s.span<=span_max).collect();
    if stations.len()<2{return Err("diamond key span does not cross enough wing stations".into());}
    let mut mesh=MeshGL64{num_prop:3,..Default::default()};
    for s in &stations{let cx=s.chord*chord_fraction;let hw=if chord_fraction<0.5{5.0}else{4.0};for &(x,z) in &[(cx-hw,base_z),(cx,base_z-height),(cx+hw,base_z),(cx,base_z+height)]{mesh.vert_properties.extend(transform_station(s,x,z));}}
    close_loft(&mut mesh,stations.len());let solid=Manifold::from_mesh_gl64(&mesh);if solid.status().to_str()!="No Error"{return Err(format!("invalid diamond key: {}",solid.status()).into());}Ok(solid)
}

fn chord_region(spec:&WingSpec,z_min:f64,z_max:f64,chord_margin:f64)->Result<Manifold,Box<dyn std::error::Error>>{let mut mesh=MeshGL64{num_prop:3,..Default::default()};for s in &spec.stations{for &(x,z)in &[(-chord_margin,z_min),(s.chord+chord_margin,z_min),(s.chord+chord_margin,z_max),(-chord_margin,z_max)]{mesh.vert_properties.extend(transform_station(s,x,z));}}close_loft(&mut mesh,spec.stations.len());let solid=Manifold::from_mesh_gl64(&mesh);if solid.status().to_str()!="No Error"{return Err(format!("invalid chord region: {}",solid.status()).into());}Ok(solid)}
fn close_loft(mesh:&mut MeshGL64,stations:usize){for s in 0..stations-1{let a=(s*4)as u64;let b=((s+1)*4)as u64;for i in 0..4_u64{let j=(i+1)%4;mesh.tri_verts.extend([a+i,b+i,b+j]);mesh.tri_verts.extend([a+i,b+j,a+j]);}}mesh.tri_verts.extend([0,1,2,0,2,3]);let e=((stations-1)*4)as u64;mesh.tri_verts.extend([e,e+2,e+1,e,e+3,e+2]);}
fn transform_station(s:&WingStation,x:f64,z:f64)->[f64;3]{let p=0.25*s.chord;let a=s.twist_deg.to_radians();let dx=x-p;[p+dx*a.cos()+z*a.sin()+s.x_offset,s.span,-dx*a.sin()+z*a.cos()+s.z_offset]}
