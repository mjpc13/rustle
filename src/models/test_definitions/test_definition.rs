use core::fmt;
use std::path::Display;

use chrono::Utc;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

use super::{speed::SpeedTestParams, CutParams, DropParams};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TestType {
    Simple,
    Speed(SpeedTestParams),
    #[serde(rename = "drop")]
    Drop(DropParams),
    #[serde(rename = "cut")]
    Cut(CutParams)
}

#[derive(Debug, Deserialize)]
pub struct TestDefinitionsConfig {
    pub test_definitions: Vec<TestDefinition>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestDefinition {
    pub id: Option<Thing>,
    pub name: String,
    pub workers: u8,
    pub iterations: u8,
    pub dataset_name: String,
    pub algo_list: Vec<String>,
    #[serde(rename = "test_type")]
    pub test_type: TestType,
    #[serde(default = "Utc::now")]
    pub created_at: chrono::DateTime<Utc>,
    #[serde(default = "Utc::now")]
    pub updated_at: chrono::DateTime<Utc>,
}


#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Sensor {
    Imu,
    Lidar,
    Camera,
    Radar,
    Gps
    // Add other sensors as needed
}

impl fmt::Display for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self{
            Sensor::Imu => write!(f, "imu"),
            Sensor::Lidar => write!(f, "lidar"),
            Sensor::Camera => write!(f, "camera"),
            Sensor::Radar => write!(f, "radar"),
            Sensor::Gps => write!(f, "gps"),
        }
    }
}
