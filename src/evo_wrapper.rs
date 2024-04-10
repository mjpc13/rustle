use core::f32;
use std::process::Command;
use std::fmt;
use log::warn;
use struct_iterable::Iterable;

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
    pose_relation: PoseRelation,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
    align: bool,
    scale: bool,
    n_to_align: Option<f32>,
    align_origin: bool,
    //plot: PlotArg
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
                        Some(val) => commands.push(format!(" --{s} {val}")),
                        None => (),
                    }
                } else if o.is::<bool>(){ //This handles all the bools
                    let val = o.downcast_ref::<bool>().unwrap();
                    if *val{
                        match s{
                            "align" => commands.push(" -a".to_string()),
                            "scale" => commands.push(" -s".to_string()),
                            "align_origin" => commands.push(" --align_origin"),
                            _ => (),
                        }
                    }
                } else if o.is::<PoseRelation>(){ 
                    match o.downcast_ref::<PoseRelation>().unwrap(){
                        PoseRelation::Full => commands.push(" -r full"),
                        PoseRelation::TransPart => commands.push(" -r trans_part"),
                        PoseRelation::RotPart => commands.push(" -r rot_part"),
                        PoseRelation::Angle(AngleUnit::Degree) => commands.push(" -r angle_deg"),
                        PoseRelation::Angle(AngleUnit::Radian) => commands.push(" -r angle_rad"),
                        PoseRelation::PointDistance => commands.push(" -r point_distance"),
                        _ => (),
                    }
                } else{
                    println!("WHAT THE HELLLL")
                }

            }).collect();
        
        todo!();
    }
}

impl Default for EvoArgs {
    fn default() -> EvoArgs {
        EvoArgs{
            t_max_diff: None,
            t_offset: None,
            t_start: None,
            t_end: None,
            pose_relation: PoseRelation::Full,// full,trans_part,rot_part,angle_deg,angle_rad,point_distance
            align: true,
            scale: true,
            n_to_align: None,
            align_origin: true,
            //plot: PlotArg::default()
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
                    println!("WHAT THE HELLLL")
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

pub struct PlotArg{
    mode: PlotMode,//xy,xz,yx,yz,zx,zy,xyz
    x_dimension: AxisUnit,//index,seconds,distances
    colormap_max: Option<u32>,
    colormap_min: Option<u32>,
    colormap_max_percentile: Option<u32>, //overrides plot_colormap_max
    path: String,//
}

impl Default for PlotArg {
    fn default() -> PlotArg {
        PlotArg{
            mode: PlotMode::XYZ,//xy,xz,yx,yz,zx,zy,xyz
            x_dimension: AxisUnit::Second,//index,seconds,distances
            colormap_max: None,
            colormap_min: None,
            colormap_max_percentile: None, //overrides plot_colormap_max
            path: String::from("SomeString"),//
        }
    }
}


pub fn evo(groundtruth: &str, data: &str, args: EvoArgs){
    let output = Command::new("evo_ape")
        .arg("tum")
        .arg(groundtruth)
        .arg(data)
        //.args(args)
        .output()
        .unwrap();

    println!("STDOUT {}", String::from_utf8_lossy(&output.stdout));
    println!("========================================");
    println!("STDERR {}", String::from_utf8_lossy(&output.stderr));

    //To parse the result I must subdivide with /n (the second index will be the Aligment,then max, min,...)

}