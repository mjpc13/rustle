use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::metric::{MetricTypeInfo, StatisticalMetrics};
use surrealdb::sql::Thing;


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseErrorMetrics {
    pub ape: StatisticalMetrics,
    pub rpe: StatisticalMetrics,
    #[serde(rename = "created_at")] // Match SurrealDB's field name
    pub created_at: DateTime<Utc>
    //pub trajectory_length: f64,
}

impl MetricTypeInfo for PoseErrorMetrics {
    fn type_name(&self) -> &'static str { "pose_error" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}


impl PoseErrorMetrics {
    pub fn mean(metrics: &[&Self]) -> Option<PoseErrorMetrics> {
        if metrics.is_empty() {
            return None;
        }
    
        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);
    
    
        Some(PoseErrorMetrics {
            ape: StatisticalMetrics::mean(&metrics.iter().map(|m| &m.ape).collect::<Vec<_>>()),
            rpe: StatisticalMetrics::mean(&metrics.iter().map(|m| &m.rpe).collect::<Vec<_>>()),
            created_at,
        })
    }
}