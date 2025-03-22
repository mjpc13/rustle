use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;


#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TestExecutionStatus {
    Scheduled,
    Running,
    Completed,
    Failed(String), // Error message for failure
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestExecution {
    pub id: Option<Thing>,                   // Format: "test_execution:<ulid>"
    //pub test_definition_id: String,   // Reference to TestDefinition
    //pub dataset_id: String,           // Reference to Dataset
    pub status: TestExecutionStatus,
    pub start_time: Option<DateTime<Utc>>,
    pub end_time: Option<DateTime<Utc>>,
    //pub environment: Environment,
    //pub parameters: serde_json::Value,// Test-specific parameters
    pub results: Option<TestResults>, 
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Environment {
    pub os: String,
    pub ros_version: String,
    pub hardware_id: Option<String>,  // Optional hardware identifier
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestResults {
    pub metrics: Vec<TestMetric>,
    pub log_uri: String,              // Path/URI to full logs
    pub artifacts: Vec<String>,       // URIs to output files
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestMetric {
    pub name: String,
    pub value: f64,
    pub unit: Option<String>,
}

impl TestExecution {
    pub fn new(
    ) -> Self {
        Self {
            id: None,
            status: TestExecutionStatus::Scheduled,
            start_time: Some(Utc::now()),
            end_time: None,
            results: None,
        }
    }
}