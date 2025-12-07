use std::cmp::Ordering;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::services::error::MetricError;

use super::{metric::{MetricTypeInfo, StatisticalMetrics}, ContainerStats};


#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
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
            Vec::new(),
            |mut u, stat| {
                let mem = &stat.memory_stats;
                
                // Memory usage in MB
                u.push(mem.usage.ok_or(MetricError::MissingError("Missing limit memory".to_owned())).unwrap() as f32 / 1e6);
                
                u
            },
        );

        // Usage trend calculation
        let time_points: Vec<i64> = stats.iter()
            .map(|s| s.created_at.timestamp())
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

fn linear_regression_slope(x: &[i64], y: &[f32]) -> f32 {
    assert_eq!(x.len(), y.len(), "x and y must be the same length");
    let n = x.len();

    // Normalize x to prevent floating point precision issues
    let x0 = x[0];
    let norm_x: Vec<i64> = x.iter().map(|xi| xi - x0).collect();

    let sum_x: i64 = norm_x.iter().sum();
    let sum_y: f32 = y.iter().sum();
    let sum_xy: f32 = norm_x.iter().zip(y).map(|(x, y)| (*x as f32) * y).sum();
    let sum_x2: f32 = norm_x.iter().map(|x| (*x as f32) * (*x as f32)).sum();
    let n_f32 = n as f32;

    let denominator = n_f32 * sum_x2 - sum_x as f32 * sum_x as f32;
    if denominator.abs() < f32::EPSILON {
        return 0.0; // Avoid division by zero or near-zero
    }

    (n_f32 * sum_xy - sum_x as f32 * sum_y) / denominator
}



impl MetricTypeInfo for MemoryMetrics {
    fn type_name(&self) -> &'static str { "memory" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::{Duration, Utc};
    use bollard::container::{CPUStats, CPUUsage, MemoryStats, ThrottlingData};

    fn make_memory_stats(usage: u64, limit: u64, created_at: DateTime<Utc>) -> ContainerStats {
        ContainerStats {
            id: None,
            memory_stats: MemoryStats {
                usage: Some(usage),
                limit: Some(limit),
                stats: None,
                max_usage: None,
                failcnt: None,
                commit: None,
                commit_peak: None,
                commitbytes: None,
                commitpeakbytes: None,
                privateworkingset: None,
            },
            cpu_stats: CPUStats {
                cpu_usage: CPUUsage {
                    total_usage: 10_000_000_000,
                    usage_in_kernelmode: 0,
                    usage_in_usermode: 0,
                    percpu_usage: None,
                },
                system_cpu_usage: None,
                online_cpus: Some(4), // 4 cores
                throttling_data: ThrottlingData {
                    periods: 0,
                    throttled_periods: 0,
                    throttled_time: 0,
                },
            },
            precpu_stats: CPUStats {
                cpu_usage: CPUUsage {
                    total_usage: 10_000_000_000 - 10_000_000,    // delta_total = 10_000_000
                    usage_in_kernelmode: 0,
                    usage_in_usermode: 0,
                    percpu_usage: None,
                },
                system_cpu_usage: None, // delta_system = 100_000_000
                online_cpus: Some(4),
                throttling_data: ThrottlingData {
                    periods: 0,
                    throttled_periods: 0,
                    throttled_time: 0,
                },
            },
            num_procs: 1,
            created_at,
        }
    }

    fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
        (a - b).abs() < epsilon
    }


    #[test]
    fn test_memory_metrics_basic_usage() {
        let now = Utc::now();
        let stats = vec![
            make_memory_stats(500_000_000, 1_000_000_000, now),
            make_memory_stats(600_000_000, 1_000_000_000, now + Duration::seconds(100)),
            make_memory_stats(700_000_000, 1_000_000_000, now + Duration::seconds(150)),
        ];

        let metrics = MemoryMetrics::from_stats(&stats).unwrap().unwrap();
        assert!(metrics.usage.mean > 0.0, "Expected positive memory mean, got {}", metrics.usage.mean);
        assert!(metrics.usage_trend_mb_sec > 0.0, "Expected positive memory trend, got {}", metrics.usage_trend_mb_sec);
        assert_eq!(metrics.limit_mb, 1_000.0);
    }

    #[test]
    fn test_memory_metrics_empty_input() {
        let stats = vec![];
        let result = MemoryMetrics::from_stats(&stats).unwrap();
        assert!(result.is_none(), "Expected None for empty input");
    }

    #[test]
    fn test_memory_metrics_missing_limit() {
        let now = Utc::now();
        let mut stats = vec![make_memory_stats(500_000_000, 1_000_000_000, now)];
        stats[0].memory_stats.limit = None;

        let result = MemoryMetrics::from_stats(&stats);
        assert!(result.is_err(), "Expected error due to missing memory limit");
    }

    #[test]
    fn test_memory_metrics_mean_aggregation() {
        let now = Utc::now();
        let stats1 = vec![
            make_memory_stats(500_000_000, 1_000_000_000, DateTime::from_timestamp(1431648000, 0).expect("invalid timestamp")),
            make_memory_stats(600_000_000, 1_000_000_000, DateTime::from_timestamp(1431648010, 0).expect("invalid timestamp")),
            make_memory_stats(750_000_000, 1_000_000_000, DateTime::from_timestamp(1431648015, 0).expect("invalid timestamp")),
        ];
        let stats2 = vec![
            make_memory_stats(550_000_000, 1_000_000_000, now),
            make_memory_stats(650_000_000, 1_000_000_000, now + Duration::seconds(10)),
        ];

        let metric1 = MemoryMetrics::from_stats(&stats1).unwrap().unwrap();
        let metric2 = MemoryMetrics::from_stats(&stats2).unwrap().unwrap();
        let mean = MemoryMetrics::mean(&[&metric1, &metric2]).unwrap();

        assert!(approx_eq(mean.limit_mb, 1000.0, 1.0), "Expected limit_mb to be ~1000");
        assert!(mean.usage.mean > 0.0, "Expected non-zero usage mean");
        assert!(mean.usage_trend_mb_sec > 0.0, "Expected average trend");
    }
}