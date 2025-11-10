use chrono::{DateTime, TimeZone, Utc};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::{models::ros::ros_msg::Ros1, services::RosError};




#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Tum {
    #[serde(deserialize_with = "from_timestamp")]
    pub time: DateTime<Utc>,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64
}

// Custom deserializer for UNIX timestamps with fractional seconds
fn from_timestamp<'de, D>(deserializer: D) -> Result<DateTime<Utc>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let s = f64::deserialize(deserializer)?;
    let secs = s.trunc() as i64;
    let nsecs = ((s.fract()) * 1e9) as u32;
    Ok(Utc.timestamp_opt(secs, nsecs)
        .single()
        .ok_or_else(|| serde::de::Error::custom("invalid timestamp"))?)
}

impl Ros1 for Tum {
    fn empty() -> Tum {
        Tum{
            time: DateTime::from_timestamp(0, 0).unwrap(),
            x: 0.0,
            y: 0.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        }
    }

    fn from_yaml(value: Yaml) -> Result<Tum, RosError>{
        let mut time = value["time"].as_str().unwrap().split(".");
        let sec = time.next().unwrap().parse::<i64>().unwrap();
        let nsec = time.next().unwrap().parse::<u32>().unwrap();
        let time = DateTime::from_timestamp(sec, nsec).unwrap();


        let x: f64 = value["x"].as_f64().unwrap();
        let y: f64 = value["y"].as_f64().unwrap();
        let z: f64 = value["z"].as_f64().unwrap();
        let qx: f64 = value["qx"].as_f64().unwrap();
        let qy: f64 = value["qy"].as_f64().unwrap();
        let qz: f64 = value["qz"].as_f64().unwrap();
        let qw: f64 = value["w"].as_f64().unwrap();
        

        Ok(
            Tum{
                time,
                x,
                y,
                z,
                qx,
                qy,
                qz,
                qw,
            }
        )
    }

    fn from_json(value: &serde_json::Value) -> Result<Tum, RosError> {
        let mut time = value["time"].as_str().unwrap().split(".");
        let sec = time.next().unwrap().parse::<i64>().unwrap();
        let nsec = time.next().unwrap().parse::<u32>().unwrap();
        let time = DateTime::from_timestamp(sec, nsec).unwrap();
        
        let x: f64 = value["x"].as_f64().unwrap();
        let y: f64 = value["y"].as_f64().unwrap();
        let z: f64 = value["z"].as_f64().unwrap();
        let qx: f64 = value["qx"].as_f64().unwrap();
        let qy: f64 = value["qy"].as_f64().unwrap();
        let qz: f64 = value["qz"].as_f64().unwrap();
        let qw: f64 = value["qw"].as_f64().unwrap();
        

        Ok(
            Tum{
                time,
                x,
                y,
                z,
                qx,
                qy,
                qz,
                qw,
            }
        )
    }

}