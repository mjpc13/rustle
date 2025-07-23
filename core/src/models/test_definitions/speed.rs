use log::warn;
use serde::{Deserialize, Serialize, Deserializer};
use serde::de::{self};



#[derive(Debug, Clone, Serialize)]
pub struct SpeedTestParams {
    pub speed_factors: Vec<f32>,
    pub speed_range: f32,
    pub speed_step: f32,
}

impl<'de> Deserialize<'de> for SpeedTestParams {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Debug, Deserialize)]
        struct RawParams {
            #[serde(default)]
            speed_factors: Option<Vec<f32>>,
            #[serde(default)]
            speed_range: Option<f32>,
            #[serde(default)]
            speed_step: Option<f32>,
        }

        let raw = RawParams::deserialize(deserializer)?;


        match (raw.speed_factors, raw.speed_range, raw.speed_step) {
            (Some(mut factors), _, _) => {
                if factors.is_empty() {
                    return Err(de::Error::custom("speed_factors must not be empty"));
                }

                // Insert 1.0 if missing (with epsilon tolerance)
                if !factors.iter().any(|&f| (f - 1.0).abs() < f32::EPSILON) {
                    factors.push(1.0);
                }

                Ok(SpeedTestParams {
                    speed_factors: factors.clone(),
                    speed_range: *factors.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap_or(&1.0),
                    speed_step: 0.0,
                })
            }
            (None, Some(range), Some(step)) => {
                if step <= 0.0 {
                    return Err(de::Error::custom("speed_step must be greater than 0"));
                }

                let mut factors = Vec::new();
                let mut current = 1.0;
                while current <= range {
                    factors.push(current);
                    current += step;
                }

                Ok(SpeedTestParams {
                    speed_factors: factors,
                    speed_range: range,
                    speed_step: step,
                })
            }
            _ => Err(de::Error::custom(
                "Must provide either `speed_factors` or both `speed_range` and `speed_step`",
            )),
        }
    }
}