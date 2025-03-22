use chrono::{DateTime, Duration, Utc};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use serde_with::serde_as;

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
    
}

impl Ros1 for Pose {
    fn empty() -> Pose{
        Pose { 
            position: Point3::origin(), 
            orientation: Quaternion::identity(), 
            covariance: None 
        }
    }
}

impl Ros1 for PoseStamped {
    fn empty() -> PoseStamped{
        PoseStamped{
            header: Header::empty(),
            pose: Pose::empty()
        }
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
}
