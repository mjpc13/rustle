use serde::{Serialize, Deserialize};
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MetricType {
    Ape(StatisticalMetrics),
    Rpe(StatisticalMetrics),
    OutputFrequency(SingleValueMetric),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatisticalMetrics {
    pub rmse: f64,
    pub mean: f64,
    pub median: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub sse: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleValueMetric {
    pub value: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metric {
    pub id: String,              // Format: "metric:<ulid>"
    pub test_run_id: String,     // Reference to TestRun
    pub algorithm_id: String,    // Reference to Algorithm
    pub metric_type: MetricType,
    pub timestamp: DateTime<Utc>,
    pub additional_info: Option<serde_json::Value>,
}