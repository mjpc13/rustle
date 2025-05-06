use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use surrealdb::sql::Thing;

use super::{metric::Metric, metrics::PoseErrorMetrics};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmRun {
    pub id: Option<Thing>,
    pub bag_speed: f32,
    pub num_iterations: u8,
    pub metrics: Vec<Metric>, 
    pub created_at: DateTime<Utc>,
    pub duration_secs: f64,
}

impl AlgorithmRun {
    pub fn new(bag_speed: f32, num_iterations: u8) -> Self {

        Self{
            id: None,
            bag_speed,
            created_at: Utc::now(),
            num_iterations,
            duration_secs: 0.0,
            metrics: Vec::new()
        }

    }
}