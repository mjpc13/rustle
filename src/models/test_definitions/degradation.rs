use std::collections::HashMap;

use chrono::Utc;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationParams {
    pub degradations: Vec<Degradation>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Degradation {
    pub sensor: Sensor,
    #[serde(flatten)]  // Flattens DegradationMode fields into this struct
    pub mode: DegradationMode,
    pub topic: String,
    #[serde(default)]
    pub active_periods: Vec<ActivePeriod>,
}



impl DegradationParams {

    //pub fn to_ros_params(&self) -> String {
    //    let mut output = HashMap::new();
    //    let mut topics = HashMap::new();
    //    for degradation in &self.degradations {
    //        let sensor_key = degradation.sensor.to_string().to_lowercase();
    //        let mut args = match &degradation.mode {
    //            DegradationMode::Frequency { drop_rate } =    pub fn to_ros_params(&self) -> String {
    //                let mut output = HashMap::new();
    //                let mut topics = HashMap::new();
    //        
    //                for degradation in &self.degradations {
    //                    let sensor_key = degradation.sensor.to_string().to_lowercase();
    //                    let mut args = match &degradation.mode {
    //                        DegradationMode::Frequency { drop_rate } => {
    //                            // Reverse for ROS format: [n, m] = drop m every n messages
    //                            vec![drop_rate[1], drop_rate[0]]
    //                        },
    //                        DegradationMode::Cutoff => {
    //                            degradation.active_periods.iter()
    //                                .flat_map(|p| vec![p.start_time, p.duration])
    //                                .collect()
    //                        }
    //                    };
    //        
    //                    // Special case: if no active periods for cutoff, default to full cutoff
    //                    if args.is_empty() && matches!(degradation.mode, DegradationMode::Cutoff) {
    //                        args = vec![0, u32::MAX]; // Full duration cutoff
    //                    }
    //        
    //                    topics.insert(
    //                        sensor_key,
    //                        HashMap::from([
    //                            ("input".to_string(), degradation.topic.clone()),
    //                            ("drop_args".to_string(), args)
    //                        ])
    //                    );
    //                };
    //        
    //                let mut final_map = HashMap::new();
    //                final_map.insert("topics_to_drop", topics);
    //                
    //                serde_yaml::to_string(&final_map)
    //                    .unwrap_or_else(|_| String::from("Error generating YAML"))
    //            }> {
    //                // Reverse for ROS format: [n, m] = drop m every n messages
    //                vec![drop_rate[1], drop_rate[0]]
    //            },
    //            DegradationMode::Cutoff => {
    //                degradation.active_periods.iter()
    //                    .flat_map(|p| vec![p.start_time, p.duration])
    //                    .collect()
    //            }
    //        };
    //        // Special case: if no active periods for cutoff, default to full cutoff
    //        if args.is_empty() && matches!(degradation.mode, DegradationMode::Cutoff) {
    //            args = vec![0, u32::MAX]; // Full duration cutoff
    //        }
    //        topics.insert(
    //            sensor_key,
    //            HashMap::from([
    //                ("input".to_string(), degradation.topic.clone()),
    //                ("drop_args".to_string(), args)
    //            ])
    //        );
    //    }
    //    let mut final_map = HashMap::new();
    //    final_map.insert("topics_to_drop", topics);
    //    
    //    serde_yaml::to_string(&final_map)
    //        .unwrap_or_else(|_| String::from("Error generating YAML"))
    //}

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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum DegradationMode {
    Frequency {
        drop_rate: [u32; 2],
    },
    Cutoff,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivePeriod {
    #[serde(rename = "start_time")]
    pub start_sec: u32,
    #[serde(rename = "duration")]
    pub duration_sec: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repeat: Option<RepeatConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepeatConfig {
    #[serde(rename = "interval_sec")]
    pub interval: u32,
    #[serde(rename = "count")]
    pub repetitions: u32,
}