use chrono::Utc;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorDegradationParams {
    pub degradations: Vec<Degradation>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Degradation {
    pub sensor: Sensor,
    pub mode: DegradationMode,
    #[serde(default)]
    pub active_periods: Vec<ActivePeriod>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Sensor {
    Imu,
    Lidar,
    Camera,
    Radar,
    // Add other sensors as needed
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "mode")]
pub enum DegradationMode {
    Frequency {
        #[serde(rename = "cut_rate")]
        rate_factor: f32,
    },
    Cutoff,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivePeriod {
    #[serde(rename = "start_time")]
    pub start_sec: u32,
    pub duration_sec: u32,
}