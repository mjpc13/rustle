use std::cmp::Ordering;

use bollard::container::MemoryStats;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::services::error::MetricError;

use super::{metric::{MetricTypeInfo, StatisticalMetrics}, ContainerStats};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CpuMetrics {
    pub load: StatisticalMetrics,
    pub throttling: StatisticalMetrics,
    #[serde(rename = "created_at")] // Match SurrealDB's field name
    pub created_at: DateTime<Utc>
}

impl MetricTypeInfo for CpuMetrics {
    fn type_name(&self) -> &'static str { "cpu" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}

impl CpuMetrics {

    pub fn from_stats(stats: &Vec<ContainerStats>) -> Option<Self> {
        let mut cpu_percentages = Vec::new();
        let mut throttling_percentages = Vec::new();

        // Calculate CPU percentages between consecutive stats
        for stat in stats {

            // Handle Option values for system_cpu_usage
            let (prev_system, current_system) = match (
                stat.precpu_stats.system_cpu_usage,
                stat.cpu_stats.system_cpu_usage,
            ) {
                (Some(p), Some(c)) => (p, c),
                _ => continue,
            };

            // Calculate deltas
            let delta_system = current_system - prev_system;
            let delta_total = stat.cpu_stats.cpu_usage.total_usage
                .checked_sub(stat.precpu_stats.cpu_usage.total_usage)
                .unwrap_or(0) as f32;

            if delta_system == 0 {
                continue; // Skip invalid data points
            }

            let num_cores = match stat.cpu_stats.online_cpus {
                Some(n) => n,
                _ => continue,
            };

            let cpu_percent = (delta_total as f32 / delta_system as f32) * 100.0 * num_cores as f32;
            cpu_percentages.push(cpu_percent);

            // Calculate throttling percentage
            let delta_periods = stat.cpu_stats.throttling_data.periods
                - stat.precpu_stats.throttling_data.periods;
            let delta_throttled = stat.cpu_stats.throttling_data.throttled_periods
                - stat.precpu_stats.throttling_data.throttled_periods;

            if delta_periods > 0 {
                let throttling_pct = (delta_throttled as f32 / delta_periods as f32) * 100.0 * num_cores as f32;
                throttling_percentages.push(throttling_pct);
            }
        }

        if cpu_percentages.is_empty() {
            return None;
        }

        
        // Compute statistical metrics for both CPU and throttling
        let load_metrics = Self::compute_statistical_metrics(&cpu_percentages);
        let throttling_metrics = Self::compute_statistical_metrics(&throttling_percentages);

        Some(Self {
            load:load_metrics,
            throttling: throttling_metrics,
            created_at: stats.last()?.created_at,
        })
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


    pub fn mean(metrics: &[&Self]) -> Option<CpuMetrics> {

        if metrics.is_empty() {
            return None;
        }

        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);

        // Collect all stats references
        let load_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.load).collect();
        let throttle_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.throttling).collect();

        Some(CpuMetrics {
            load: StatisticalMetrics::mean(&load_list).ok_or(MetricError::ComputeError("Could not compute mean of cpu load".to_owned())).unwrap(),
            throttling: StatisticalMetrics::mean(&throttle_list).ok_or(MetricError::ComputeError("Could not compute mean of cpu throttle".to_owned())).unwrap(),
            created_at,
        })
    }

}
#[cfg(test)]
mod tests {
    use super::*;
    use bollard::container::{CPUStats, CPUUsage, ThrottlingData};
    use chrono::{DateTime, Utc};

    fn make_cpu_stats(
        total_usage: u64,
        system_cpu_usage: u64,
        throttling_periods: u64,
        throttled_periods: u64,
        created_at: DateTime<Utc>,
    ) -> ContainerStats {
        ContainerStats {
            id: None,
            memory_stats: MemoryStats {
                stats: None,
                max_usage: None,
                usage: None,
                failcnt: None,
                limit: None,
                commit: None,
                commit_peak: None,
                commitbytes: None,
                commitpeakbytes: None,
                privateworkingset: None,
            },
            cpu_stats: CPUStats {
                cpu_usage: CPUUsage {
                    total_usage,
                    usage_in_kernelmode: 0,
                    usage_in_usermode: 0,
                    percpu_usage: None,
                },
                system_cpu_usage: Some(system_cpu_usage),
                online_cpus: Some(4), // 4 cores
                throttling_data: ThrottlingData {
                    periods: throttling_periods,
                    throttled_periods,
                    throttled_time: 0,
                },
            },
            precpu_stats: CPUStats {
                cpu_usage: CPUUsage {
                    total_usage: total_usage - 10_000_000,    // delta_total = 10_000_000
                    usage_in_kernelmode: 0,
                    usage_in_usermode: 0,
                    percpu_usage: None,
                },
                system_cpu_usage: Some(system_cpu_usage - 100_000_000), // delta_system = 100_000_000
                online_cpus: Some(4),
                throttling_data: ThrottlingData {
                    periods: throttling_periods,
                    throttled_periods,
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
    fn test_cpu_metrics_basic_usage() {
        let now = Utc::now();
        let stats = vec![
            make_cpu_stats(10_000_000_000, 200_000_000_000_000, 10, 2, now),
            make_cpu_stats(11_000_000_000, 200_001_000_000_000, 20, 5, now),
        ];

        let metrics = CpuMetrics::from_stats(&stats).expect("Expected valid CpuMetrics");

        // Check CPU load values are within expected bounds
        assert!(
            approx_eq(metrics.load.mean, 40.0, 5.0),
            "Expected load.mean ≈ 400, got {}",
            metrics.load.mean
        );

        // Check throttling % calculation is roughly correct
        assert!(
            metrics.throttling.mean >= 0.0,
            "Expected throttling to be >= 0, got {}",
            metrics.throttling.mean
        );

        // Check created_at is from last stat
        assert_eq!(metrics.created_at, now);
    }

    #[test]
    fn test_cpu_metrics_empty_input() {
        let stats = vec![];
        let metrics = CpuMetrics::from_stats(&stats);
        assert!(metrics.is_none(), "Expected None for empty stats");
    }

    #[test]
    fn test_cpu_metrics_missing_system_cpu_usage() {
        let now = Utc::now();
        let mut stat = make_cpu_stats(10_000_000, 200_000_000_000_000, 0, 0, now);
        stat.cpu_stats.system_cpu_usage = None; // Break the data
        let stats = vec![stat];

        let metrics = CpuMetrics::from_stats(&stats);
        assert!(metrics.is_none(), "Expected None due to missing system_cpu_usage");
    }

    #[test]
    fn test_cpu_metrics_mean_aggregation() {
        let now = Utc::now();
        let stats = vec![
            make_cpu_stats(10_000_000_000, 200_000_000_000_000, 10, 2, now),
            make_cpu_stats(11_000_000_000, 200_001_000_000_000, 20, 4, now),
        ];
        let metric1 = CpuMetrics::from_stats(&stats).unwrap();
        let metric2 = CpuMetrics::from_stats(&stats).unwrap();
        let mean = CpuMetrics::mean(&[&metric1, &metric2]).unwrap();

        assert!(
            approx_eq(mean.load.mean, metric1.load.mean, 1e-4),
            "Mean CPU load should match input average"
        );
        assert_eq!(mean.created_at, now);
    }
}