// models/ros/msg.rs

use super::error::RosError;
use chrono::{DateTime, Duration, Utc};
use nalgebra::{Point3, Quaternion, Vector3, Matrix6};

// Your original message structs go here
#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Pose { /* ... */ }

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Odometry { /* ... */ }

// Keep original implementations
impl std::fmt::Display for Odometry { /* ... */ }
impl Sub for Header { /* ... */ }