use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use std::{
    clone::Clone, 
    cmp::PartialEq,
    fmt::Debug
};

use nalgebra::{
    base::{Vector3, Matrix6},
    geometry::{Point3, Quaternion}
};

#[derive(Debug, Serialize, Deserialize)]
pub struct Header{
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>
}

// #[derive(Debug, Clone)]
#[derive(Debug, Serialize, Deserialize)]
pub struct Pose<R, S> 
where
    R: PartialEq + Clone + Debug + 'static,
    S: Debug + Clone + PartialEq + 'static,
{
    position: Point3<R>,
    orientation: Quaternion<S>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Twist<T>
where
    T: PartialEq + Clone + Debug + 'static,
{
    linear: Vector3<T>,
    angular: Vector3<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Odometry<S, T> 
where
    S: PartialEq + Clone + Debug + 'static,
    T: Debug + Clone + PartialEq + 'static
{
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Pose<S, T>,
    pub twist: Twist<T>
}
