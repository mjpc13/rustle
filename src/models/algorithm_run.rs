use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use crate::models::{Algorithm, ros::Odometry};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmRun {
    pub id: String,                 // Format: "algorithm_run:<ulid>"
    //pub test_execution_id: String,  // Reference to TestExecution
    //pub algorithm_id: String,       // Reference to Algorithm
    pub iterations: Vec<RunIteration>,
    pub aggregates: RunAggregates,
    pub created_at: DateTime<Utc>,
    pub duration_secs: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunIteration {
    pub iteration_num: u32,
    pub ape: f64,
    pub rpe: f64,
    pub cpu_usage: f32,
    pub memory_usage_mb: f32,
    pub odometry_estimates: Vec<Odometry>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunAggregates {
    pub avg_ape: f64,
    pub avg_rpe: f64,
    pub max_cpu: f32,
    pub avg_memory: f32,
    pub total_estimates: usize,
}

impl AlgorithmRun {
    pub fn new(test_execution_id: &str, algorithm_id: &str) -> Self {
        Self {
            id: format!("algorithm_run:{}", ulid::Ulid::new()),
            //test_execution_id: test_execution_id.to_string(),
            //algorithm_id: algorithm_id.to_string(),
            iterations: Vec::new(),
            aggregates: RunAggregates::default(),
            created_at: Utc::now(),
            duration_secs: 0.0,
        }
    }
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