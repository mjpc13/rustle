use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::metric::StatisticalMetrics;
use surrealdb::sql::Thing;


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseErrorMetrics {
    pub ape: StatisticalMetrics,
    pub rpe: StatisticalMetrics,
    pub created_at: DateTime<Utc>
    //pub trajectory_length: f64,
}