use std::num::NonZeroUsize;

use mold_core::{Axis, SectionedTwoPartMold};
use mold_geometry::{Bounds3, SolidKernel, Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShellSettings {
    pub thickness: f64,
    pub flange_width: f64,
    pub web_thickness: f64,
}

impl Default for ShellSettings {
    fn default() -> Self {
        Self { thickness: 3.0, flange_width: 12.0, web_thickness: 3.0 }
    }
}

/// Explicit geometry used to divide a shell into its two mold halves.
/// Registration sockets are subtracted after the skin and flange have been
/// united so neither can accidentally fill a socket intended for a loose key.
pub struct PartingRegions<'a, S> {
    pub negative: &'a S,
    pub positive: &'a S,
    pub negative_flange: Option<&'a S>,
    pub positive_flange: Option<&'a S>,
    pub negative_sockets: &'a [&'a S],
    pub positive_sockets: &'a [&'a S],
}

pub fn generate_sectioned_shell_mold<K>(kernel: &K, part: &K::Solid, split_axis: Axis, section_axis: Axis, section_count: NonZeroUsize, settings: ShellSettings) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where K: SolidKernel {
    let part_bounds = kernel.bounds(part)?;
    let expanded = kernel.offset(part, settings.thickness)?;
    let skin = kernel.difference(&expanded, part)?;
    let expanded_bounds = kernel.bounds(&expanded)?;
    let split = midpoint(part_bounds, split_axis);
    let (negative_half, positive_half) = split_bounds(expanded_bounds, split_axis, split);
    let sections = section_ranges(expanded_bounds, section_axis, section_count);
    let negative = generate_half(kernel, part, &skin, part_bounds, negative_half, split_axis, section_axis, split, &sections, settings, Half::Negative)?;
    let positive = generate_half(kernel, part, &skin, part_bounds, positive_half, split_axis, section_axis, split, &sections, settings, Half::Positive)?;
    Ok(SectionedTwoPartMold { negative, positive })
}

pub fn generate_sectioned_shell_mold_with_parting<K>(kernel: &K, part: &K::Solid, parting: PartingRegions<'_, K::Solid>, section_axis: Axis, section_count: NonZeroUsize, settings: ShellSettings) -> Result<SectionedTwoPartMold<K::Solid>, K::Error>
where K: SolidKernel {
    let expanded = kernel.offset(part, settings.thickness)?;
    let skin = kernel.difference(&expanded, part)?;
    let expanded_bounds = kernel.bounds(&expanded)?;
    let sections = section_ranges(expanded_bounds, section_axis, section_count);
    let mut negative_skin = kernel.intersection(&skin, parting.negative)?;
    let mut positive_skin = kernel.intersection(&skin, parting.positive)?;
    if let Some(flange) = parting.negative_flange { negative_skin = kernel.union(&negative_skin, &kernel.difference(flange, part)?)?; }
    if let Some(flange) = parting.positive_flange { positive_skin = kernel.union(&positive_skin, &kernel.difference(flange, part)?)?; }
    for socket in parting.negative_sockets { negative_skin = kernel.difference(&negative_skin, socket)?; }
    for socket in parting.positive_sockets { positive_skin = kernel.difference(&positive_skin, socket)?; }
    let negative = clip_sections(kernel, &negative_skin, expanded_bounds, section_axis, &sections)?;
    let positive = clip_sections(kernel, &positive_skin, expanded_bounds, section_axis, &sections)?;
    Ok(SectionedTwoPartMold { negative, positive })
}

fn clip_sections<K>(kernel: &K, solid: &K::Solid, bounds: Bounds3, section_axis: Axis, sections: &[(f64, f64)]) -> Result<Vec<K::Solid>, K::Error>
where K: SolidKernel {
    let mut pieces = Vec::with_capacity(sections.len());
    for &(section_min, section_max) in sections { let mut clip=bounds;set_min(&mut clip,section_axis,section_min);set_max(&mut clip,section_axis,section_max);pieces.push(kernel.intersection(solid,&kernel.cuboid(clip)?)?); }
    Ok(pieces)
}

#[derive(Debug, Clone, Copy)] enum Half { Negative, Positive }
#[allow(clippy::too_many_arguments)]
fn generate_half<K>(kernel:&K,part:&K::Solid,skin:&K::Solid,part_bounds:Bounds3,half_bounds:Bounds3,split_axis:Axis,section_axis:Axis,split:f64,section_ranges:&[(f64,f64)],settings:ShellSettings,half:Half)->Result<Vec<K::Solid>,K::Error> where K:SolidKernel { let mut pieces=Vec::with_capacity(section_ranges.len());for &(section_min,section_max) in section_ranges{let mut clip=half_bounds;set_min(&mut clip,section_axis,section_min);set_max(&mut clip,section_axis,section_max);let mut piece=kernel.intersection(skin,&kernel.cuboid(clip)?)?;piece=union_web(kernel,part,piece,split_web_bounds(part_bounds,clip,split_axis,section_axis,split,settings,half))?;piece=union_web(kernel,part,piece,section_web_bounds(part_bounds,clip,split_axis,section_axis,section_min,settings,WebSide::Min))?;piece=union_web(kernel,part,piece,section_web_bounds(part_bounds,clip,split_axis,section_axis,section_max,settings,WebSide::Max))?;pieces.push(piece);}Ok(pieces)}
fn union_web<K:SolidKernel>(kernel:&K,part:&K::Solid,piece:K::Solid,bounds:Bounds3)->Result<K::Solid,K::Error>{let web=kernel.difference(&kernel.cuboid(bounds)?,part)?;kernel.union(&piece,&web)}
fn split_web_bounds(part_bounds:Bounds3,section_bounds:Bounds3,split_axis:Axis,section_axis:Axis,split:f64,settings:ShellSettings,half:Half)->Bounds3{let mut b=padded(part_bounds,settings.flange_width);copy_axis_range(&mut b,section_bounds,section_axis);match half{Half::Negative=>{set_min(&mut b,split_axis,split-settings.web_thickness);set_max(&mut b,split_axis,split)},Half::Positive=>{set_min(&mut b,split_axis,split);set_max(&mut b,split_axis,split+settings.web_thickness)}}b}
#[derive(Debug,Clone,Copy)]enum WebSide{Min,Max}
fn section_web_bounds(part_bounds:Bounds3,section_bounds:Bounds3,split_axis:Axis,section_axis:Axis,interface:f64,settings:ShellSettings,side:WebSide)->Bounds3{let mut b=padded(part_bounds,settings.flange_width);let(hmin,hmax)=axis_range(section_bounds,split_axis);set_min(&mut b,split_axis,hmin);set_max(&mut b,split_axis,hmax);match side{WebSide::Min=>{set_min(&mut b,section_axis,interface);set_max(&mut b,section_axis,interface+settings.web_thickness)},WebSide::Max=>{set_min(&mut b,section_axis,interface-settings.web_thickness);set_max(&mut b,section_axis,interface)}}b}
fn padded(b:Bounds3,a:f64)->Bounds3{Bounds3{min:Vec3::new(b.min.x-a,b.min.y-a,b.min.z-a),max:Vec3::new(b.max.x+a,b.max.y+a,b.max.z+a)}}
fn midpoint(b:Bounds3,a:Axis)->f64{let(m,n)=axis_range(b,a);(m+n)*0.5}
fn split_bounds(b:Bounds3,a:Axis,s:f64)->(Bounds3,Bounds3){let mut n=b;let mut p=b;set_max(&mut n,a,s);set_min(&mut p,a,s);(n,p)}
fn section_ranges(b:Bounds3,a:Axis,c:NonZeroUsize)->Vec<(f64,f64)>{let(min,max)=axis_range(b,a);let w=(max-min)/c.get() as f64;(0..c.get()).map(|i|{let s=min+i as f64*w;let e=if i+1==c.get(){max}else{min+(i+1)as f64*w};(s,e)}).collect()}
fn copy_axis_range(t:&mut Bounds3,s:Bounds3,a:Axis){let(min,max)=axis_range(s,a);set_min(t,a,min);set_max(t,a,max)}
fn axis_range(b:Bounds3,a:Axis)->(f64,f64){match a{Axis::X=>(b.min.x,b.max.x),Axis::Y=>(b.min.y,b.max.y),Axis::Z=>(b.min.z,b.max.z)}}
fn set_min(b:&mut Bounds3,a:Axis,v:f64){match a{Axis::X=>b.min.x=v,Axis::Y=>b.min.y=v,Axis::Z=>b.min.z=v}}
fn set_max(b:&mut Bounds3,a:Axis,v:f64){match a{Axis::X=>b.max.x=v,Axis::Y=>b.max.y=v,Axis::Z=>b.max.z=v}}

#[cfg(test)] mod tests {use super::*;use mold_manifold::ManifoldKernel;#[test]fn cuboid_shell_is_thinner_than_a_solid_block(){let k=ManifoldKernel;let p=k.cuboid(Bounds3{min:Vec3::new(-20.0,-50.0,-5.0),max:Vec3::new(20.0,50.0,5.0)}).unwrap();let m=generate_sectioned_shell_mold(&k,&p,Axis::Z,Axis::Y,NonZeroUsize::new(2).unwrap(),ShellSettings::default()).unwrap();assert_eq!(m.negative.len(),2);assert_eq!(m.positive.len(),2);for p in m.negative.iter().chain(&m.positive){assert_eq!(p.0.status().to_str(),"No Error");assert!(!p.0.is_empty());assert!(p.0.volume()>0.0);}}}
