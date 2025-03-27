use chrono::Utc;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

//use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TestType {
    Simple,
    Speed(SpeedTestParams),
    //Drop(DropTestParams),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimpleTestParams {}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedTestParams {
    pub speed_factors: Vec<f32>,
}

#[derive(Debug, Deserialize)]
pub struct TestDefinitionsConfig {
    pub test_definitions: Vec<TestDefinition>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestDefinition {
    pub id: Option<Thing>,
    pub name: String,
    pub workers: u8,
    pub iterations: u8,
    pub dataset_name: String,
    pub algo_list: Vec<String>,
    #[serde(rename = "test_type")]
    pub test_type: TestType,
    #[serde(default = "Utc::now")]
    pub created_at: chrono::DateTime<Utc>,
    #[serde(default = "Utc::now")]
    pub updated_at: chrono::DateTime<Utc>,
}

//#[derive(Debug, Clone, Serialize, Deserialize)]
//pub struct DropTestParams {
//    pub topics: Vec<String>,
//    pub drop_patterns: Vec<DropPattern>,
//}
//
//#[derive(Debug, Clone, Serialize, Deserialize)]
//pub struct DropPattern {
//    pub percentage: f32,
//    pub duration_ms: u64,
//}