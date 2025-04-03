use std::cmp::Ordering;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use super::{metric::{MetricTypeInfo, StatisticalMetrics}, ContainerStats};

#[derive(Debug, Clone, Serialize, Deserialize)]
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
                .unwrap_or(0) as f64;

            if delta_system == 0 {
                continue; // Skip invalid data points
            }

            let num_cores = match stat.cpu_stats.online_cpus {
                Some(n) => n,
                _ => continue,
            };

            let cpu_percent = (delta_total as f64 / delta_system as f64) * 100.0 * num_cores as f64;
            cpu_percentages.push(cpu_percent);

            // Calculate throttling percentage
            let delta_periods = stat.cpu_stats.throttling_data.periods
                - stat.precpu_stats.throttling_data.periods;
            let delta_throttled = stat.cpu_stats.throttling_data.throttled_periods
                - stat.precpu_stats.throttling_data.throttled_periods;

            if delta_periods > 0 {
                let throttling_pct = (delta_throttled as f64 / delta_periods as f64) * 100.0 * num_cores as f64;
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

    fn compute_statistical_metrics(data: &[f64]) -> StatisticalMetrics {
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

        let mean = data.iter().sum::<f64>() / data.len() as f64;
        
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
            .sum::<f64>() / data.len() as f64;
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

        let count = metrics.len() as f64;
        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);

        // Collect all stats references
        let load_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.load).collect();
        let throttle_list: Vec<&StatisticalMetrics> = metrics.iter().map(|m| &m.throttling).collect();

        Some(CpuMetrics {
            load: StatisticalMetrics::mean(&load_list),
            throttling: StatisticalMetrics::mean(&throttle_list),
            created_at,
        })
    }

}