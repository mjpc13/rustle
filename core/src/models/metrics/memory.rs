use std::cmp::Ordering;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::services::error::MetricError;

use super::{metric::{MetricTypeInfo, StatisticalMetrics}, ContainerStats};


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemoryMetrics {
    pub limit_mb: f32,
    pub usage: StatisticalMetrics,
    pub usage_trend_mb_sec: f32,
    #[serde(rename = "created_at")] // Match SurrealDB's field name
    pub created_at: DateTime<Utc>
}

impl MemoryMetrics {
    pub fn from_stats(stats: &[ContainerStats]) -> Result<Option<Self>, MetricError> {
        if stats.is_empty() {
            return Ok(None);
        }

        let limit = stats[0]
            .memory_stats.limit
            .ok_or(MetricError::MissingError("Missing limit memory".to_owned()))? as f32;

        let limit_mb = limit / 1e6;

        // Raw value collections
        let usage_values = stats.iter()
        .fold(
            (Vec::new()),
            |(mut u), stat| {
                let mem = &stat.memory_stats;
                
                // Memory usage in MB
                u.push(mem.usage.ok_or(MetricError::MissingError("Missing limit memory".to_owned())).unwrap() as f32 / 1e6);
                
                u
            },
        );

        // Usage trend calculation
        let time_points: Vec<f32> = stats.iter()
            .map(|s| s.created_at.timestamp() as f32)
            .collect();
        let usage_trend = linear_regression_slope(&time_points, &usage_values);

        Ok(Some(Self {
            limit_mb,
            usage_trend_mb_sec: usage_trend,
            created_at: Utc::now(),
            usage: compute_statistical_metrics(&usage_values),
        }))
    }



    pub fn mean(metrics: &[&Self]) -> Option<MemoryMetrics> {

        if metrics.is_empty() {
            return None;
        }

        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);

        // Collect all stats references
        let load_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.usage).collect();
        let usage_trend_list: Vec<&f32> = metrics.iter().map(|m| &m.usage_trend_mb_sec).collect();
        let limit_mb = metrics.iter().next().ok_or(MetricError::MissingError("Missing memory limit".to_owned())).unwrap().limit_mb;


        // Convert to Vec<f32> by dereferencing
        let values: Vec<f32> = usage_trend_list.into_iter().copied().collect();

        // Compute mean
        let mean = if !values.is_empty() {
            Some(values.iter().sum::<f32>() / values.len() as f32)
        } else {
            None
        };

        let usage_trend = mean.ok_or(MetricError::ComputeError("Unable to compute mean of usage trend".to_owned())).unwrap();

        Some(MemoryMetrics{
            usage: StatisticalMetrics::mean(&load_list).ok_or(MetricError::ComputeError("Could not compute mean of cpu load".to_owned())).unwrap(),
            created_at,
            limit_mb: limit_mb,
            usage_trend_mb_sec: usage_trend,
        })
    }




}

fn compute_statistical_metrics(data: &[f32]) -> StatisticalMetrics {
    if data.is_empty() {
        return StatisticalMetrics {
            mean: 0.0,
            median: 0.0,
            min: 0.0,
            max: 0.0,
            std: 0.0,
            rmse: None,
            sse: None,
        };
    }

    let mean = data.iter().sum::<f32>() / data.len() as f32;
    
    let mut sorted = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));

    let median = if sorted.len() % 2 == 0 {
        let mid = sorted.len() / 2;
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[sorted.len() / 2]
    };

    let min = *sorted.first().unwrap();
    let max = *sorted.last().unwrap();

    let variance = data.iter()
        .map(|x| (x - mean).powi(2))
        .sum::<f32>() / data.len() as f32;
    let std = variance.sqrt();

    StatisticalMetrics {
        mean,
        median,
        min,
        max,
        std,
        rmse: None,
        sse: None,
    }
}

// Helper function for trend calculation
fn linear_regression_slope(x: &[f32], y: &[f32]) -> f32 {
    // Implementation of simple linear regression
    let n = x.len() as f32;
    let sum_x: f32 = x.iter().sum();
    let sum_y: f32 = y.iter().sum();
    let sum_xy: f32 = x.iter().zip(y).map(|(x, y)| x * y).sum();
    let sum_x2: f32 = x.iter().map(|x| x * x).sum();
    
    (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
}

impl MetricTypeInfo for MemoryMetrics {
    fn type_name(&self) -> &'static str { "memory" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}
