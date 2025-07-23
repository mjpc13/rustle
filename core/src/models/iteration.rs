use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

use crate::models::{TestExecution, TestType};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Iteration {
    pub id: Option<Thing>,
    pub iteration_num: u8,
    pub test_type: TestType,
    pub container: DockerContainer,
    pub created_at: DateTime<Utc>,
    pub exec_id: Thing
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockerContainer{
    pub image_name: String,
    pub container_name: String,
}

impl Iteration {
    pub fn new(iteration_num: u8, container: DockerContainer, test_type: TestType, exec: TestExecution) -> Self {

        Self {
            id: None,
            iteration_num,
            container,
            test_type,
            created_at: Utc::now(),
            exec_id: exec.id.unwrap(),
        }
    }
}