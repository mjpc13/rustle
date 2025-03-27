use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmRun {
    pub id: Option<Thing>,
    pub bag_speed: f32,
    pub num_iterations: u8,
    pub aggregates: RunAggregates,
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
            aggregates: RunAggregates::default()
        }

    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunAggregates {
    pub avg_ape: f64,
    pub avg_rpe: f64,
    pub max_cpu: f32,
    pub avg_memory: f32,
    pub total_estimates: usize,
}

impl Default for RunAggregates {
    fn default() -> Self {
        Self {
            avg_ape: 0.0,
            avg_rpe: 0.0,
            max_cpu: 0.0,
            avg_memory: 0.0,
            total_estimates: 0,
        }
    }
}