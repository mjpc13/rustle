use core::f32;
use std::process::Command;
use std::fmt;
use log::{warn, trace};
use struct_iterable::Iterable;

use crate::{errors::{EvoError, RosError}, metrics::Metrics};

enum AngleUnit{
    Degree,
    Radian
}

enum PoseRelation{
    Full,
    TransPart,
    RotPart,
    Angle(AngleUnit),
    PointDistance
}

#[derive(Iterable)]
pub struct EvoArgs{
    t_max_diff: Option<f32>,
    t_offset: Option<f32>,
    t_start: Option<f32>,
    t_end: Option<f32>,
    pose_relation: Option<PoseRelation>,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
    align: bool,
    scale: bool,
    n_to_align: Option<f32>,
    align_origin: bool,
    plot: Option<PlotArg>
}

impl EvoArgs {
    fn get_commands(&self) -> Vec<String>{
        
        let mut commands: Vec<String> = Vec::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<f32>
                if o.is::<Option<f32>>(){
                    match o.downcast_ref::<Option<f32>>().unwrap(){
                        Some(val) => commands.push(format!("--{s} {val}")),
                        None => (),
                    }
                } else if o.is::<bool>(){ //This handles all the bools
                    let val = o.downcast_ref::<bool>().unwrap();
                    if *val{
                        match s{
                            "align" => commands.push("-a".to_string()),
                            "scale" => commands.push("-s".to_string()),
                            "align_origin" => commands.push("--align_origin".to_string()),
                            _ => (),
                        }
                    }
                } else if o.is::<Option<PoseRelation>>(){ 
                    match o.downcast_ref::<Option<PoseRelation>>().unwrap(){
                        Some(val) => {
                            match val{
                                PoseRelation::Full => commands.push("-r full".to_string()),
                                PoseRelation::TransPart => commands.push("-r trans_part".to_string()),
                                PoseRelation::RotPart => commands.push("-r rot_part".to_string()),
                                PoseRelation::Angle(AngleUnit::Degree) => commands.push("-r angle_deg".to_string()),
                                PoseRelation::Angle(AngleUnit::Radian) => commands.push("-r angle_rad".to_string()),
                                PoseRelation::PointDistance => commands.push("-r point_distance".to_string())                            }
                        },
                        None => (),
                    }

                } else{
                    println!("{s:}");
                }
            }).collect();
        
        match &self.plot{
            Some(p) =>{
                let plot_cmd: Vec<String> = p.get_commands();
                commands.extend(plot_cmd);
            },
            None => ()
        }

        return commands;
    }
}

impl Default for EvoArgs {
    fn default() -> EvoArgs {
        EvoArgs{
            t_max_diff: None,
            t_offset: None,
            t_start: None,
            t_end: None,
            pose_relation: None,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
            align: true,
            scale: true,
            n_to_align: None,
            align_origin: true,
            plot: None
        }
    }
}

impl fmt::Display for EvoArgs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {

        let mut string = String::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<f32>
                if o.is::<Option<f32>>(){
                    match o.downcast_ref::<Option<f32>>().unwrap(){
                        Some(val) => string.push_str(&format!(" --{s} {val}")),
                        None => (),
                    }
                } else if o.is::<bool>(){ //This handles all the bools
                    let val = o.downcast_ref::<bool>().unwrap();
                    if *val{
                        match s{
                            "align" => string.push_str(" -a"),
                            "scale" => string.push_str(" -s"),
                            "align_origin" => string.push_str(" --align_origin"),
                            _ => (),
                        }
                    }
                } else if o.is::<PoseRelation>(){ 
                    match o.downcast_ref::<PoseRelation>().unwrap(){
                        PoseRelation::Full => string.push_str(" -r full"),
                        PoseRelation::TransPart => string.push_str(" -r trans_part"),
                        PoseRelation::RotPart => string.push_str(" -r rot_part"),
                        PoseRelation::Angle(AngleUnit::Degree) => string.push_str(" -r angle_deg"),
                        PoseRelation::Angle(AngleUnit::Radian) => string.push_str(" -r angle_rad"),
                        PoseRelation::PointDistance => string.push_str(" -r point_distance"),
                        _ => (),
                    }
                } else{
                    println!("{s:}")
                }

            }).collect();

        write!(f, "{}", string)
    }
}

enum AxisUnit{
    Index,
    Second,
    Distance
}

enum PlotMode{
    XY,
    XZ,
    YX,
    YZ,
    ZX,
    ZY,
    XYZ
}

#[derive(Iterable)]
pub struct PlotArg{
    mode: PlotMode,//xy,xz,yx,yz,zx,zy,xyz
    x_dimension: AxisUnit,//index,seconds,distances
    colormap_max: Option<u32>,
    colormap_min: Option<u32>,
    colormap_max_percentile: Option<u32>, //overrides plot_colormap_max
    path: Option<String>,//
}

impl PlotArg {
    fn get_commands(&self) -> Vec<String>{
        
        let mut commands: Vec<String> = Vec::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<u32>
                if o.is::<Option<u32>>(){
                    match o.downcast_ref::<Option<u32>>().unwrap(){
                        Some(val) => commands.push(format!(" --plot_{s} {val}")),
                        None => (),
                    }
                } else if o.is::<String>(){ //This handles all the bools
                    match o.downcast_ref::<Option<String>>().unwrap(){
                        Some(val) => commands.push(val.to_string()),
                        None => (),
                    }
                } else if o.is::<PlotMode>(){ 
                    match o.downcast_ref::<PlotMode>().unwrap(){
                        PlotMode::XY => commands.push(" --plot_mode=xy".to_string()),
                        PlotMode::XZ => commands.push(" --plot_mode=xz".to_string()),
                        PlotMode::YX => commands.push(" --plot_mode=yx".to_string()),
                        PlotMode::YZ => commands.push(" --plot_mode=yz".to_string()),
                        PlotMode::ZX => commands.push(" --plot_mode=zx".to_string()),
                        PlotMode::ZY => commands.push(" --plot_mode=zy".to_string()),
                        PlotMode::XYZ => commands.push(" --plot_mode=xyz".to_string()),
                        _ => (),
                    }
                } else if o.is::<AxisUnit>(){ 
                    match o.downcast_ref::<AxisUnit>().unwrap(){
                        AxisUnit::Index => commands.push(" --plot_x_dimention index".to_string()),
                        AxisUnit::Second => commands.push(" --plot_x_dimention second".to_string()),
                        AxisUnit::Distance => commands.push(" --plot_x_dimention distance".to_string()),
                        _ => (),
                    }
                } else{
                    println!("WHAT THE HELLLL")
                }
            }).collect();
        
        return commands;
    }
}

impl Default for PlotArg {
    fn default() -> PlotArg {
        PlotArg{
            mode: PlotMode::XYZ,//xy,xz,yx,yz,zx,zy,xyz
            x_dimension: AxisUnit::Second,//index,seconds,distances
            colormap_max: None,
            colormap_min: None,
            colormap_max_percentile: None, //overrides plot_colormap_max
            path: None,//
        }
    }
}


pub fn evo_ape(groundtruth: &str, data: &str, args: EvoArgs) -> Result<Metrics, EvoError>{
    let output = Command::new("evo_ape")
        .arg("tum")
        .arg(groundtruth)
        .arg(data)
        .args(args.get_commands())
        .output()
        .unwrap();

    let stdout = std::str::from_utf8(&output.stdout).unwrap();
    let stderr = std::str::from_utf8(&output.stderr).unwrap();
    
    trace!("========================================");
    trace!("STDOUT {}", stdout);
    trace!("========================================");
    trace!("STDERR {}", stderr);
    trace!("========================================");

    if stderr.is_empty() && !stdout.is_empty(){

        return Ok(Metrics::parse(stdout)?);

        //parse
    } else{
        return Err(EvoError::CommandError { stderr: stderr.into() });
    }
}
