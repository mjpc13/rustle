use chrono::Utc;
use serde::{Serialize, Deserialize};
//use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TestType {
    Simple(SimpleTestParams),
    Speed(SpeedTestParams),
    //Drop(DropTestParams),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimpleTestParams {
    pub iterations: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedTestParams {
    pub speed_factors: Vec<f32>,
    pub max_allowed_delay: Option<f32>,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestDefinition {
    pub id: String,
    pub name: String,
    pub test_type: TestType,
    pub created_at: chrono::DateTime<Utc>,
    pub updated_at: chrono::DateTime<Utc>,
}