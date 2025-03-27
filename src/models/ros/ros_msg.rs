use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;


use crate::services::error::RosError;

use super::{Header, Pose, PoseStamped, Twist, Path, Odometry};

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
                        id: None,
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
                        id: None,
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
