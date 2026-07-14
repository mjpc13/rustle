use std::fmt;

use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::RosVersion;
use super::{header::Header, ros_msg::RosData, pose::Pose, twist::Twist};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Odometry {
    pub id: Option<Thing>,
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Option<Pose>,
    pub twist: Option<Twist>,
    pub created_at: DateTime<Utc>
}

impl Odometry {
    pub fn new(header: Header) -> Self {
        Self {
            id: None,
            header,
            child_frame_id: None,
            pose: None,
            twist: None,
            created_at: Utc::now()
        }
    }
}

impl RosData for Odometry {
    fn empty() -> Odometry{
        Odometry{
            id: None,
            header: Header::empty(),
            child_frame_id: None,
            pose: None,
            twist: None,
            created_at: Utc::now()
        }
    }

    fn from_yaml(yaml: Yaml, ros_version: RosVersion) -> Result<Odometry, RosError>{
        let header = Header::from_yaml(yaml["header"].clone(), ros_version)?;

        let pose = match Pose::from_yaml(yaml["pose"]["pose"].clone(), ros_version){
            Ok(p) => Some(p),
            Err(_) => None
        };
        let twist = match Twist::from_yaml(yaml["twist"]["twist"].clone(), ros_version){
            Ok(t) => Some(t),
            Err(_) => None
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
            child_frame_id,
            created_at: Utc::now()
            }
        )
    }
    fn from_json(value: &serde_json::Value, ros_version: RosVersion) -> Result<Odometry, RosError> {
        // Parse header
        let header = Header::from_json(&value["header"], ros_version)?;

        // Parse pose
        let pose = match value["pose"]["pose"].as_object() {
            Some(_) => Some(Pose::from_json(&value["pose"]["pose"], ros_version)?),
            None => None,
        };

        // Parse twist
        let twist = match value["twist"]["twist"].as_object() {
            Some(_) => Some(Twist::from_json(&value["twist"]["twist"], ros_version)?),
            None => None,
        };

        // Parse child_frame_id
        let child_frame_id = value["child_frame_id"]
            .as_str()
            .map(|s| s.to_string());

        Ok(Odometry {
            id: None,
            header,
            pose,
            twist,
            child_frame_id,
            created_at: Utc::now(),
        })
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
