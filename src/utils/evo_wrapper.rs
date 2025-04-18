use core::f64;
use std::process::Command;
use std::fmt;
use log::{debug, error, info, trace, warn};
use struct_iterable::Iterable;

use crate::services::error::{EvoError, RosError};

pub trait EvoArg {
    fn compute<'a>(&self, groundtruth: &str, data: &str) -> Result<String, EvoError>;
}

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
pub struct EvoApeArg{
    pub t_max_diff: Option<f32>,
    pub t_offset: Option<f32>,
    pub t_start: Option<f32>,
    pub t_end: Option<f32>,
    pub pose_relation: Option<PoseRelation>,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
    pub align: bool,
    pub scale: bool,
    pub n_to_align: Option<f32>,
    pub plot: Option<PlotArg>
}

impl EvoApeArg {
    fn get_commands(&self) -> Vec<String>{
        
        let mut commands: Vec<String> = Vec::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<f32>
                if o.is::<Option<f32>>(){
                    match o.downcast_ref::<Option<f32>>().unwrap(){
                        Some(val) => commands.push(format!("--{s}={val}")),
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
                    //println!("{s:}");
                }
            }).collect();
        
        match &self.plot{
            Some(p) =>{
                let plot_cmd: Vec<String> = p.get_commands("ape");
                commands.extend(plot_cmd);
            },
            None => ()
        }

        debug!("Flags for EVO APE:{:#?}", commands);

        return commands;
    }
}

impl Default for EvoApeArg {
    fn default() -> EvoApeArg {
        EvoApeArg{
            t_max_diff: None,
            t_offset: None,
            t_start: None,
            t_end: None,
            pose_relation: None,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
            align: true,
            scale: true,
            n_to_align: None,
            plot: None
        }
    }
}

impl fmt::Display for EvoApeArg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {

        let mut string = String::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<f32>
                if o.is::<Option<f32>>(){
                    match o.downcast_ref::<Option<f32>>().unwrap(){
                        Some(val) => string.push_str(&format!(" --{s}={val}")),
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
    pub mode: PlotMode,//xy,xz,yx,yz,zx,zy,xyz
    pub x_dimension: AxisUnit,//index,seconds,distances
    pub colormap_max: Option<u32>,
    pub colormap_min: Option<u32>,
    pub colormap_max_percentile: Option<u32>, //overrides plot_colormap_max
    pub path: String,//
}

impl PlotArg {
    fn get_commands(&self, metric_type: &str) -> Vec<String>{
        
        let mut commands: Vec<String> = Vec::new();
        commands.push(format!("--save_plot={}/{metric_type}.pdf", self.path));
        //commands.push("-p".to_owned());


        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<u32>
                if o.is::<Option<u32>>(){
                    match o.downcast_ref::<Option<u32>>().unwrap(){
                        Some(val) => commands.push(format!("--plot_{s} {val}")),
                        None => (),
                    }
                } else if o.is::<PlotMode>(){ 
                    match o.downcast_ref::<PlotMode>().unwrap(){
                        PlotMode::XY => commands.push("--plot_mode=xy".to_string()),
                        PlotMode::XZ => commands.push("--plot_mode=xz".to_string()),
                        PlotMode::YX => commands.push("--plot_mode=yx".to_string()),
                        PlotMode::YZ => commands.push("--plot_mode=yz".to_string()),
                        PlotMode::ZX => commands.push("--plot_mode=zx".to_string()),
                        PlotMode::ZY => commands.push("--plot_mode=zy".to_string()),
                        PlotMode::XYZ => commands.push("--plot_mode=xyz".to_string()),
                        _ => (),
                    }
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
            path: "None".to_owned(),//
        }
    }
}

impl EvoArg for EvoApeArg{
    fn compute<'a>(&self, groundtruth: &str, data: &str) -> Result<String, EvoError>{

        warn!("My gt: {groundtruth} and data: {data}");

        let output = Command::new("evo_ape")
            .arg("tum")
            .arg(groundtruth)
            .arg(data)
            .args(self.get_commands())
            .output()
            .unwrap();
    
        let stdout = std::str::from_utf8(&output.stdout).unwrap();
        let stderr = std::str::from_utf8(&output.stderr).unwrap();

        if stdout.contains("[ERROR]"){
            return Err(EvoError::CommandError(stdout.into()));
            
        } else if !stderr.is_empty(){
            log::warn!("The evo_ape command gave an error. This might be a bad parameter setup or a bad file location");
            return Err(EvoError::CommandError(stderr.into()));

        } else if !stdout.is_empty(){
            
            return Ok(stdout.to_string());

        } else{
            return Err(EvoError::CommandError(stderr.into()));
        }
    }
    
}


#[derive(Iterable)]
pub struct EvoRpeArg{
    pub t_max_diff: Option<f32>,
    pub t_offset: Option<f32>,
    pub t_start: Option<f32>,
    pub t_end: Option<f32>,
    pub pose_relation: Option<PoseRelation>,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
    pub align: bool,
    pub scale: bool,
    pub n_to_align: Option<f32>,
    pub plot: Option<PlotArg>
}


impl EvoRpeArg {
    fn get_commands(&self) -> Vec<String>{
        
        let mut commands: Vec<String> = Vec::new();

        let test: Vec<_> = self
            .iter()
            .map( | (s, o) | {
                
                //This handles all the Option<f32>
                if o.is::<Option<f32>>(){
                    match o.downcast_ref::<Option<f32>>().unwrap(){
                        Some(val) => commands.push(format!("--{s}={val}")),
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
                let plot_cmd: Vec<String> = p.get_commands("rpe");
                commands.extend(plot_cmd);
            },
            None => ()
        }

        return commands;
    }
}

impl Default for EvoRpeArg {
    fn default() -> EvoRpeArg {
        EvoRpeArg{
            t_max_diff: None,
            t_offset: None,
            t_start: None,
            t_end: None,
            pose_relation: None,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
            align: true,
            scale: true,
            n_to_align: None,
            plot: None
        }
    }
}

impl EvoArg for EvoRpeArg{
    fn compute<'a>(&self, groundtruth: &str, data: &str) -> Result<String, EvoError>{        
        let output = Command::new("evo_rpe")
            .arg("tum")
            .arg(groundtruth)
            .arg(data)
            .args(self.get_commands())
            .output()
            .unwrap();
    
        let stdout = std::str::from_utf8(&output.stdout).unwrap();
        let stderr = std::str::from_utf8(&output.stderr).unwrap();

        if stdout.contains("[ERROR]"){
            return Err(EvoError::CommandError(stdout.into()));
            
        } else if !stderr.is_empty(){
            log::warn!("The evo_ape command gave an error. This might be a bad parameter setup or a bad file location");
            return Err(EvoError::CommandError(stderr.into()));

        } else if !stdout.is_empty(){
            
            return Ok(stdout.to_string());

        } else{
            return Err(EvoError::CommandError(stderr.into()));
        }
    }
    
}
