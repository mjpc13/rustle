use serde::{Deserialize, Serialize};



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpeedTestParams {
    pub speed_factors: Vec<f32>,
}