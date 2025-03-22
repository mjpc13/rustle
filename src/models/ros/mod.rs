use serde::{Serialize, Deserialize};
use chrono::{DateTime, Utc};
use nalgebra::{Point3, Quaternion, Vector3, Matrix6};


// Make sure RosMsg is properly declared and exported
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum RosMsg {
    Header(Header),
    Pose(Pose),
    PoseStamped(PoseStamped),
    Twist(Twist),
    Path(Path),
    Odometry(Odometry)
}


#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Odometry {
    // Database fields
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub test_run_id: Option<String>,
    
    // Original ROS1 fields
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Option<Pose>,
    pub twist: Option<Twist>,
    
    // Processing metadata
    #[serde(default = "Utc::now")]
    pub stored_at: DateTime<Utc>,
}

// Rest of original ROS message structs unchanged
#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Header {
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Pose {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub covariance: Option<Matrix6<f64>>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Twist {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub covariance: Option<Matrix6<f64>>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Path
{
    pub header: Header,
    pub poses: Vec<PoseStamped>
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct PoseStamped
{
    header: Header,
    pose: Pose
}


// Keep original Display implementation
impl std::fmt::Display for Odometry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.header.time)
    }
}