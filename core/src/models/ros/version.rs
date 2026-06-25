use std::str::FromStr;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum RosVersion {
    #[default]
    #[serde(rename = "ROS_1")]
    Ros1,
    #[serde(rename = "ROS_2")]
    Ros2,
}

impl FromStr for RosVersion {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_uppercase().trim() {
            "ROS_1" => { Ok(Self::Ros1) }
            "ROS_2" => { Ok(Self::Ros2) }
            _ => { Err(format!("Invalide ROS version: '{}', expected 'ROS_1' or ROS_2'", s)) }
        }
    }
}
