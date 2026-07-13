use chrono::Utc;
use nalgebra::{Point3, Quaternion};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;


use crate::{models::ros::Tum, services::error::RosError};

use super::{RosVersion, Header, Pose, PoseStamped, Twist, Path, Odometry};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum RosMsg{
    Header(Header),
    Pose(Pose),
    PoseStamped(PoseStamped),
    Twist(Twist),
    Path(Path),
    Odometry(Odometry),
    Tum(Tum)
}

pub trait RosData: Sized{
    fn empty() -> Self;
    fn from_yaml(yaml: Yaml, ros_version: RosVersion) -> Result<Self, RosError>;
    fn from_json(value: &serde_json::Value, ros_version: RosVersion) -> Result<Self, RosError>;
}

impl RosMsg{
    pub fn new(top_fields: Vec<&str>) -> Result<RosMsg, RosError>{

        if top_fields.contains(&"twist"){
            return Ok(RosMsg::Odometry(Odometry::empty()));
        } else if top_fields.contains(&"orientation") && top_fields.contains(&"position")  {
            return Ok(RosMsg::Pose(Pose::empty()));
        } else if top_fields.contains(&"poses")  {
            return Ok(RosMsg::Path(Path::empty()));
        } else if top_fields.contains(&"pose") {
            return Ok(RosMsg::PoseStamped(PoseStamped::empty()))
        } else {
            return Err(RosError::ParseError{from: format!("{:?}",top_fields).into(), to: "Header.timestamp".into()})
        }

    }

    pub fn as_odometry(self) -> Result<Odometry, RosError>{

        let odom = match self {
            RosMsg::Header(_v) => Err(RosError::ParseError { from: "Header".into(), to: "Odometry".into() }),
            RosMsg::Pose(_) => Err(RosError::ParseError { from: "Pose".into(), to: "Odometry".into() }),
            RosMsg::PoseStamped(p) => {
                Ok(
                    Odometry{
                        id: None,
                        header: p.header,
                        child_frame_id: None,
                        pose: Some(p.pose),
                        twist: None,
                        created_at: Utc::now()
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
                        twist: None,
                        created_at: Utc::now()
                    }
                )
            },
            RosMsg::Odometry(o) => Ok(o),
            RosMsg::Tum(t) => Ok(
                Odometry { 
                    id: None, 
                    header: Header { 
                        seq: 0, 
                        time: t.time, 
                        frame_id: None
                    }, 
                    child_frame_id: None, 
                    pose: Some( Pose{
                        position: Point3::new(t.x, t.y, t.z),
                        orientation: Quaternion::new(t.qw, t.qx, t.qy, t.qz),
                        covariance: None,
                    }), 
                    twist: None, 
                    created_at: Utc::now()
                }
            ),
        };
        odom

    }

    pub fn get_header(self) -> Result<Header, RosError> {
        match self {
            RosMsg::Header(header) => Ok(header),
            RosMsg::Pose(_pose) => Err(RosError::MissingHeader { rostype: "Pose".into() }),
            RosMsg::PoseStamped(pose_stamped) => Ok(pose_stamped.header),
            RosMsg::Twist(_twist) => Err(RosError::MissingHeader { rostype: "Twist".into() }),
            RosMsg::Path(path) => Ok(path.header),
            RosMsg::Odometry(odometry) => Ok(odometry.header),
            RosMsg::Tum(tum) => Ok(
                Header { 
                        seq: 0, 
                        time: tum.time, 
                        frame_id: None
                    }
            ),
        }
    }

    pub fn from_yaml(&self, yaml: Yaml, ros_version: RosVersion) -> Result<RosMsg, RosError>{
        match self{
            RosMsg::Header(_) => {
                Ok(RosMsg::Header(
                    Header::from_yaml(yaml, ros_version)?
                ))
            },
            RosMsg::Pose(_) => Ok(
                RosMsg::Pose(
                    Pose::from_yaml(yaml, ros_version)?
                )
            ),
            RosMsg::PoseStamped(_) => Ok(
                RosMsg::PoseStamped(
                    PoseStamped::from_yaml(yaml, ros_version)?
                )
            ),
            RosMsg::Twist(_) => Ok(
                RosMsg::Twist(
                    Twist::from_yaml(yaml, ros_version)?
                )
            ),
            RosMsg::Path(_) => Ok(
                RosMsg::Path(
                    Path::from_yaml(yaml, ros_version)?
                )
            ),
            RosMsg::Odometry(_) => Ok(
                RosMsg::Odometry(
                    Odometry::from_yaml(yaml, ros_version)?
                )
            ),
            RosMsg::Tum(_) => Ok(
                RosMsg::Tum(Tum::from_yaml(yaml, ros_version)?)
            )
        }
    }



    pub fn from_json(&self, value: &serde_json::Value, ros_version: RosVersion) -> Result<RosMsg, RosError> {
        match self {
            RosMsg::Header(_) => Ok(RosMsg::Header(
                Header::from_json(value, ros_version)?
            )),
            RosMsg::Pose(_) => Ok(RosMsg::Pose(
                Pose::from_json(value, ros_version)?
            )),
            RosMsg::PoseStamped(_) => Ok(RosMsg::PoseStamped(
                PoseStamped::from_json(value, ros_version)?
            )),
            RosMsg::Twist(_) => Ok(RosMsg::Twist(
                Twist::from_json(value, ros_version)?
            )),
            RosMsg::Path(_) => Ok(RosMsg::Path(
                Path::from_json(value, ros_version)?
            )),
            RosMsg::Odometry(_) => Ok(RosMsg::Odometry(
                Odometry::from_json(value, ros_version)?
            )),
            RosMsg::Tum(_) => Ok(
                RosMsg::Tum(Tum::from_json(value, ros_version)?)
            )
        }
    }






}
