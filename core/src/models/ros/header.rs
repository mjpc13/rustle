use std::ops::Sub;

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::ros_msg::Ros1;

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Header {
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>,
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