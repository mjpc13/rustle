use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::metric::{MetricTypeInfo, StatisticalMetrics};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpuMetrics {
    pub stats: StatisticalMetrics,
    pub idle_percent: f64,
    pub load_avg: (f64, f64, f64), // 1/5/15 minute averages
    #[serde(rename = "created_at")] // Match SurrealDB's field name
    pub created_at: DateTime<Utc>
}

impl MetricTypeInfo for CpuMetrics {
    fn type_name(&self) -> &'static str { "cpu" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}

impl CpuMetrics {
    pub fn mean(metrics: &[&Self]) -> Option<CpuMetrics> {

        if metrics.is_empty() {
            return None;
        }

        let count = metrics.len() as f64;
        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);

        // Collect all stats references
        let stats_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.stats).collect();

        Some(CpuMetrics {
            stats: StatisticalMetrics::mean(&stats_list),
            idle_percent: metrics.iter().map(|m| m.idle_percent).sum::<f64>() / count,
            load_avg: (
                metrics.iter().map(|m| m.load_avg.0).sum::<f64>() / count,
                metrics.iter().map(|m| m.load_avg.1).sum::<f64>() / count,
                metrics.iter().map(|m| m.load_avg.2).sum::<f64>() / count,
            ),
            created_at,
        })
    }

}