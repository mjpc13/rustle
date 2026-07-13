use std::ops::Sub;

use chrono::{DateTime, Duration, Utc};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::RosVersion;
use super::ros_msg::RosData;

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Header {
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>,
}

impl RosData for Header {

    fn empty() -> Header{
        Header { 
            seq: 0, 
            time: DateTime::from_timestamp(0, 0).unwrap(),
            frame_id: None
        }
    }


    fn from_yaml(yaml: Yaml, ros_version: RosVersion) -> Result<Header, RosError>{

        let seq = match ros_version {
            RosVersion::Ros1 => { yaml["seq"].as_i64() }
            RosVersion::Ros2 => { Some(0) }
        }.ok_or(RosError::FormatError(format!("Missing or invalid 'seq' in Header: {:?}", yaml)))?;

        let frame_id = match yaml["frame_id"].as_str() {
            Some("") => { None }
            Some(x) => { Some(String::from(x)) }
            None => { return Err(RosError::FormatError(format!("Missing 'frame_id' in Header: {:?}", yaml))) }
        };

        let stamp_sec = match ros_version {
            RosVersion::Ros1 => { yaml["stamp"]["secs"].as_i64() }
            RosVersion::Ros2 => { yaml["stamp"]["sec"].as_i64() }
        }.ok_or(RosError::FormatError(format!("Missing 'stamp.secs' in Header: {:?}", yaml)))?;
        let stamp_nsec = match ros_version {
            RosVersion::Ros1 => { yaml["stamp"]["nsecs"].as_i64()}
            RosVersion::Ros2 => { yaml["stamp"]["nanosec"].as_i64()}
        }.ok_or(RosError::FormatError(format!("Missing 'stamp.nsecs' in Header: {:?}", yaml)))?;

        let dt = DateTime::from_timestamp(stamp_sec, stamp_nsec as u32)
            .ok_or(RosError::FormatError(format!("Invalid timestamp: {}.{:?}", stamp_sec, stamp_nsec)))?;

        Ok(Header{
            seq: seq as u32,
            time: dt,
            frame_id
        })
    }

    fn from_json(value: &serde_json::Value, ros_version: RosVersion) -> Result<Header, RosError> {


        let seq = match ros_version {
            RosVersion::Ros1 => { value["seq"].as_i64() }
            RosVersion::Ros2 => { Some(0) }
        }.ok_or(RosError::FormatError(format!("Missing or invalid 'seq' in Header: {:?}", value)))?;

        let frame_id = match value["frame_id"].as_str() {
            Some("") => { None }
            Some(x) => { Some(String::from(x)) }
            None => { return Err(RosError::FormatError(format!("Missing 'frame_id' in Header: {:?}", value))) }
        };

        let stamp_sec = match ros_version {
            RosVersion::Ros1 => { value["stamp"]["secs"].as_i64() }
            RosVersion::Ros2 => { value["stamp"]["sec"].as_i64() }
        }.ok_or(RosError::FormatError(format!("Missing 'stamp.secs' in Header: {:?}", value)))?;
        let stamp_nsec = match ros_version {
            RosVersion::Ros1 => { value["stamp"]["nsecs"].as_i64()}
            RosVersion::Ros2 => { value["stamp"]["nanosec"].as_i64()}
        }.ok_or(RosError::FormatError(format!("Missing 'stamp.nsecs' in Header: {:?}", value)))?;

        let dt = DateTime::from_timestamp(stamp_sec, stamp_nsec as u32)
            .ok_or(RosError::FormatError(format!("Invalid timestamp: {}.{:?}", stamp_sec, stamp_nsec)))?;

        Ok(Header{
            seq: seq as u32,
            time: dt,
            frame_id
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
