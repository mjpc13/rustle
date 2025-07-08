use std::collections::HashMap;

use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

use crate::models::{metric::Metric, Algorithm};


#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TestExecutionStatus {
    Scheduled,
    Running,
    Completed,
    Failed(String), // Error message for failure
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestExecution {
    pub id: Option<Thing>,
    pub dataset_name: String,
    pub num_iterations: u8,
    pub status: TestExecutionStatus,
    pub start_time: Option<DateTime<Utc>>,
    pub end_time: Option<DateTime<Utc>>,
    pub metrics: HashMap<String, Metric>
}


impl TestExecution {
    pub fn new(num_iterations: u8, dataset_name: String) -> Self {
        Self {
            id: None,
            status: TestExecutionStatus::Scheduled,
            num_iterations,
            start_time: Some(Utc::now()),
            end_time: None,
            metrics: HashMap::new(),
            dataset_name,
        }
    }
}