use chrono::{DateTime, Duration, Utc};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use serde_with::serde_as;
use yaml_rust2::Yaml;

use std::{
    clone::Clone, 
    cmp::PartialEq,
    fmt::{self, Debug, Display}, ops::Sub
};

use nalgebra::{
    base::{Vector3, Vector4, Matrix6, Scalar},
    geometry::{Point3, Quaternion}
};

use crate::errors::RosError;

use std::fs::File;
use std::io::{self, Read};
use std::sync::Arc;
use std::str::FromStr;

use num::Num;
use num::Float;

use log::{debug, error, info, warn, trace};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum RosMsg{
    Header(Header),
    Pose(Pose),
    PoseStamped(PoseStamped),
    Twist(Twist),
    Path(Path),
    Odometry(Odometry)
}

pub trait Ros1: Sized{
    fn empty() -> Self;
    fn from_yaml(yaml: Yaml) -> Result<Self, RosError>;
}

impl RosMsg{
    pub fn new(top_fields: Vec<&str>) -> Result<RosMsg, RosError>{
        match top_fields[..]{
            [
                "position",
                "orientation"
            ] => Ok(RosMsg::Pose(Pose::empty())),
            [
                "header",
                "pose"
            ] => Ok(RosMsg::PoseStamped(PoseStamped::empty())),
            [
                "header",
                "poses",
            ] => Ok(RosMsg::Path(Path::empty())),
            [
                "header",
                "child_frame_id",
                "pose",
                "twist"
            ] => Ok(RosMsg::Odometry(Odometry::empty())),

            _ => return Err(RosError::ParseError{from: format!("{:?}",top_fields).into(), to: "Header.timestamp".into()})
        }
    }

    pub fn as_odometry(self) -> Result<Odometry, RosError>{

        let odom = match self {
            RosMsg::Header(v) => Err(RosError::ParseError { from: "Header".into(), to: "Odometry".into() }),
            RosMsg::Pose(_) => Err(RosError::ParseError { from: "Pose".into(), to: "Odometry".into() }),
            RosMsg::PoseStamped(p) => {
                Ok(
                    Odometry{
                        header: p.header,
                        child_frame_id: None,
                        pose: Some(p.pose),
                        twist: None,
                    }
                )
            },
            RosMsg::Twist(_) => Err(RosError::ParseError { from: "Twist".into(), to: "Odometry".into() }),
            RosMsg::Path(p) => {
                Ok(
                    Odometry { 
                        header: p.poses.last().unwrap().header.clone(), 
                        child_frame_id: None, 
                        pose: Some(p.poses.last().unwrap().pose.clone()), 
                        twist: None }
                )
            },
            RosMsg::Odometry(o) => Ok(o),
        };
        odom

    }

    pub fn get_header(self) -> Result<Header, RosError> {
        match self {
            RosMsg::Header(header) => Ok(header),
            RosMsg::Pose(pose) => Err(RosError::MissingHeader { rostype: "Pose".into() }),
            RosMsg::PoseStamped(pose_stamped) => Ok(pose_stamped.header),
            RosMsg::Twist(twist) => Err(RosError::MissingHeader { rostype: "Twist".into() }),
            RosMsg::Path(path) => Ok(path.header),
            RosMsg::Odometry(odometry) => Ok(odometry.header),

        }
    }


    pub fn from_yaml(&self, yaml: Yaml) -> Result<RosMsg, RosError>{
        match self{
            RosMsg::Header(_) => {
                Ok(RosMsg::Header(
                    Header::from_yaml(yaml)?
                ))
            },
            RosMsg::Pose(_) => Ok(
                RosMsg::Pose(
                    Pose::from_yaml(yaml)?
                )
            ),
            RosMsg::PoseStamped(_) => Ok(
                RosMsg::PoseStamped(
                    PoseStamped::from_yaml(yaml)?
                )
            ),
            RosMsg::Twist(_) => Ok(
                RosMsg::Twist(
                    Twist::from_yaml(yaml)?
                )
            ),
            RosMsg::Path(_) => Ok(
                RosMsg::Path(
                    Path::from_yaml(yaml)?
                )
            ),
            RosMsg::Odometry(_) => Ok(
                RosMsg::Odometry(
                    Odometry::from_yaml(yaml)?
                )
            ),
        }
    }

    fn echo(&self) -> String{
        todo!()
    }
}





#[derive(Debug, Serialize, Deserialize, Default, Clone)]
#[serde_as] // Apply the macro to the struct
pub struct Header{
    pub seq: u32,
    #[serde_as(as = "Option<TimestampSeconds<String>>")]
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>
}

// Implement the Sub trait for Header and &Header (to avoid consuming the Headers)
impl Sub for Header {
    type Output = Duration; // The result of subtraction is a Duration

    fn sub(self, other: Self) -> Duration {
        self.time - other.time
    }
}
impl Sub for &Header {
    type Output = Duration;

    fn sub(self, other: Self) -> Duration {
        self.time - other.time
    }
}




#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Pose
{
    position: Point3<f64>,
    orientation: Quaternion<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct PoseStamped
{
    header: Header,
    pose: Pose
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Path
{
    pub header: Header,
    pub poses: Vec<PoseStamped>
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Twist
{
    linear: Vector3<f64>,
    angular: Vector3<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Odometry
{
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Option<Pose>,
    pub twist: Option<Twist>
}






impl Ros1 for Header {

    fn empty() -> Header{
        Header { 
            seq: 0, 
            time: DateTime::from_timestamp(0, 0).unwrap(), 
            frame_id: None
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<Header, RosError>{

        let seq = yaml["seq"].as_i64().unwrap();
        let frame_id = yaml["frame_id"].as_str().unwrap();
        let stamp_sec = yaml["stamp"]["secs"].as_i64().unwrap();
        let stamp_nsec = yaml["stamp"]["nsecs"].as_i64().unwrap() as u32;
        let dt = DateTime::from_timestamp(stamp_sec, stamp_nsec).unwrap();

        Ok(Header{
            seq: seq as u32,
            time: dt,
            frame_id: match frame_id{
                "" => None,
                _ => Some(String::from(frame_id))
            }
        })
    }
    
}

impl Ros1 for Pose {
    fn empty() -> Pose{
        Pose { 
            position: Point3::origin(), 
            orientation: Quaternion::identity(), 
            covariance: None 
        }
    }
    fn from_yaml(yaml: Yaml) -> Result<Pose, RosError>{

        let p_x = yaml["position"]["x"].as_f64().expect(&format!("{:#?}", yaml["position"]));
        let p_y = yaml["position"]["y"].as_f64().unwrap();
        let p_z = yaml["position"]["z"].as_f64().unwrap();

        let o_x = yaml["orientation"]["x"].as_f64().unwrap();
        let o_y = yaml["orientation"]["y"].as_f64().unwrap();
        let o_z = yaml["orientation"]["z"].as_f64().unwrap();
        let o_w = yaml["orientation"]["w"].as_f64().unwrap();


        let position = Point3::from([p_x, p_y, p_z]);
        let orientation = Quaternion::from([o_x, o_y, o_z, o_w]);

        let mut pose =  Pose{
            position,
            orientation,
            covariance: None
        };

        let mut covariance: Vec<f64> = vec![];

        let _: Vec<_> = yaml["covariance"]
            .as_vec()
            .unwrap_or(
                return Ok(pose)
            )
            .iter()
            .map(|y|{
                covariance.push(y.as_f64().unwrap());
            })
            .collect();

        pose.covariance = Some(Matrix6::from_vec(covariance));

        return Ok(pose)

    }
}

impl Ros1 for PoseStamped {
    fn empty() -> PoseStamped{
        PoseStamped{
            header: Header::empty(),
            pose: Pose::empty()
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<PoseStamped, RosError>{
        
        let header = Header::from_yaml(yaml["header"].clone())?;
        let pose = Pose::from_yaml(yaml["pose"].clone())?;

        return Ok(PoseStamped{
            header,
            pose
        });

    }
}

impl Ros1 for Twist {
    fn empty() -> Twist{
        Twist { 
            linear: Vector3::zeros(), 
            angular: Vector3::zeros(), 
            covariance: None 
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<Twist, RosError>{
        let l_x = yaml["linear"]["x"].as_f64().unwrap();
        let l_y = yaml["linear"]["y"].as_f64().unwrap();
        let l_z = yaml["linear"]["z"].as_f64().unwrap();

        let a_x = yaml["angular"]["x"].as_f64().unwrap();
        let a_y = yaml["angular"]["y"].as_f64().unwrap();
        let a_z = yaml["angular"]["z"].as_f64().unwrap();

        let linear = Vector3::from([l_x, l_y, l_z]);
        let angular = Vector3::from([a_x, a_y, a_z]);

        let mut twist =  Twist{
            linear,
            angular,
            covariance: None
        };

        let mut covariance: Vec<f64> = vec![];

        let _: Vec<_> = yaml["covariance"]
            .as_vec()
            .unwrap_or(
                return Ok(twist)
            )
            .iter()
            .map(|y|{
                covariance.push(y.as_f64().unwrap());
            })
            .collect();

        twist.covariance = Some(Matrix6::from_vec(covariance));


        return Ok(twist)

    }
}

impl Ros1 for Odometry {
    fn empty() -> Odometry{
        Odometry{
            header: Header::empty(),
            child_frame_id: None,
            pose: None,
            twist: None
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<Odometry, RosError>{
        let header = Header::from_yaml(yaml["header"].clone())?;

        let pose = match Pose::from_yaml(yaml["pose"]["pose"].clone()){
            Ok(p) => Some(p),
            Err(e) => None
        };
        let twist = match Twist::from_yaml(yaml["twist"]["twist"].clone()){
            Ok(t) => Some(t),
            Err(e) => None
        };

        let child_frame_id = match yaml["child_frame_id"].as_str(){
            Some(s) => Some(s.to_string()),
            None => None
        };

        Ok(
            Odometry{
            header,
            pose,
            twist,
            child_frame_id
            }
        )

    }
}

impl fmt::Display for Odometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The odometry messages will be printed in the TUM format file
        write!(f, "{} {} {} {} {} {} {} {}",
            self.header.time.timestamp_nanos_opt().unwrap() as f64 /f64::powf(10.0,9.0), 
            self.pose.as_ref().unwrap().position[0], 
            self.pose.as_ref().unwrap().position[1], 
            self.pose.as_ref().unwrap().position[2], 
            self.pose.as_ref().unwrap().orientation.coords[0], 
            self.pose.as_ref().unwrap().orientation.coords[1], 
            self.pose.as_ref().unwrap().orientation.coords[2], 
            self.pose.as_ref().unwrap().orientation.coords[3]
        )
    }
}

impl Ros1 for Path {
    fn empty() -> Path{
        Path { 
            header: Header::empty(), 
            poses: Vec::new() 
        }
    }
    fn from_yaml(yaml: Yaml) -> Result<Path, RosError>{
        
        let header = Header::from_yaml(yaml["header"].clone())?;

        let poses: Vec<PoseStamped> = yaml["poses"].clone()
            .into_iter()
            .map(|y|{
                PoseStamped::from_yaml(y).unwrap()
            })
            .collect();

        Ok(
            Path{
                header,
                poses
            }
        )   

    }
}
