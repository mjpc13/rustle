use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use crate::models::{Odometry, RosVersion};  // Updated path
use surrealdb::sql::Thing;


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dataset {
    pub id: Option<Thing>,           // Add SurrealDB ID field
    pub name: String,
    pub dataset_path: String,
    pub ros_version: Option<RosVersion>,
    pub ground_truth_topic: Option<String>,
    pub ground_truth: Option<Vec<Odometry>>,
    pub duration: Option<f32>,
    #[serde(default = "Utc::now")]
    pub created_at: DateTime<Utc>,
}
 
impl Dataset {
    /// Pure data constructor (no I/O)
    pub fn new(name: String, dataset_path: String, ground_truth_topic: Option<String>, ros_version: Option<RosVersion>) -> Self {
        Self {
            id: None, // SurrealDB-style ID
            name,
            ground_truth_topic,
            dataset_path,
            ros_version,
            ground_truth: None,
            created_at: Utc::now(),
            duration: None,
        }
    }
}
