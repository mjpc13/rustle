use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::metric::StatisticalMetrics;
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpuMetrics {
    pub stats: StatisticalMetrics,
    pub idle_percent: f64,
    pub load_avg: (f64, f64, f64), // 1/5/15 minute averages
    pub created_at: DateTime<Utc>
}