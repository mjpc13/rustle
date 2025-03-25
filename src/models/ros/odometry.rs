use std::fmt;

use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::{header::Header, pose::Pose, ros_msg::Ros1, twist::Twist};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Odometry {
    pub id: Option<Thing>,
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Option<Pose>,
    pub twist: Option<Twist>,
}

impl Odometry {
    pub fn new(header: Header) -> Self {
        Self {
            id: None,
            header,
            child_frame_id: None,
            pose: None,
            twist: None,
        }
    }
}

impl Ros1 for Odometry {
    fn empty() -> Odometry{
        Odometry{
            id: None,
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
            id: None,
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