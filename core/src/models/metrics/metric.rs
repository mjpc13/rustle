use std::{collections::HashMap, str::FromStr};

use serde::{Deserialize, Serialize};
use itertools::Itertools;

use crate::{models::metrics::{rob::RobustnessMetric, tes::TemporalEfficiencyMetric}, services::{self}};
use surrealdb::sql::Thing;
use super::{cpu::CpuMetrics, memory::MemoryMetrics, pose_error::PoseErrorMetrics};



#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Metric {
    pub id: Option<Thing>,
    #[serde(rename = "metric_type")]
    pub metric_type: MetricType
}


impl Metric {
    pub fn mean(metrics: Vec<Metric>) -> Vec<Metric> {

        let mut result: Vec<Metric> = Vec::new();

        let mut cpu_metrics = Vec::new();
        let mut memory_metrics = Vec::new();
        let mut pose_metrics = Vec::new();
        let mut freq_metrics = Vec::new();

        // Group metrics by type
        for metric in &metrics {
            match &metric.metric_type {
                MetricType::Cpu(c) => cpu_metrics.push(c),
                MetricType::Memory(m) => memory_metrics.push(m),
                MetricType::PoseError(p) => pose_metrics.push(p),
                MetricType::Frequency(f) => freq_metrics.push(f),
                MetricType::TemporalEfficiency(_) => (),
                MetricType::Robustness(_) => (),
            }
        }

        //Compute mean of vector of PoseMetrics
        let agg_pose = PoseErrorMetrics::mean(&pose_metrics);
        if let Some(pose) = agg_pose {

            let metric = Metric {
                id: None,
                metric_type: MetricType::PoseError(pose),

            };

            result.push(metric);
        }
        
        //Compute mean of vector of CPU metrics
        let agg_cpu = CpuMetrics::mean(&cpu_metrics);
        if let Some(cpu) = agg_cpu {
            let metric = Metric {
                id: None,
                metric_type: MetricType::Cpu(cpu),

            };
            result.push(metric);        
        }
        
        //Compute mean of vector of Freq metrics
        let agg_freq = StatisticalMetrics::mean(&freq_metrics);
        if let Some(freq) = agg_freq {
            let metric = Metric {
                id: None,
                metric_type: MetricType::Frequency(freq),
            };
            result.push(metric);        
        }

        let agg_mem = MemoryMetrics::mean(&memory_metrics);
        if let Some(mem) = agg_mem{
            let metric = Metric{
                id: None,
                metric_type: MetricType::Memory(mem)
            };
            result.push(metric);
        }

        result
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "PascalCase")]
pub enum MetricType{
    Cpu(CpuMetrics),
    Memory(MemoryMetrics),
    PoseError(PoseErrorMetrics),
    Frequency(StatisticalMetrics),
    TemporalEfficiency(TemporalEfficiencyMetric),
    Robustness(RobustnessMetric)
}

impl MetricType {
    pub fn type_name(&self) -> &'static str {
        match self {
            MetricType::Cpu(_) => "cpu",
            MetricType::PoseError(_) => "pose_error",
            MetricType::Frequency(_) => "frequency",
            MetricType::Memory(_) => "memory",
            MetricType::TemporalEfficiency(_) => "temporal_efficiency",
            MetricType::Robustness(_) => "robustness",
        }
    }
    
    pub fn as_any(&self) -> &dyn std::any::Any {
        match self {
            MetricType::Cpu(m) => m,
            MetricType::PoseError(m) => m,
            MetricType::Frequency(m) => m,
            MetricType::Memory(m) => m,
            MetricType::TemporalEfficiency(m) => m,
            MetricType::Robustness(m) => m,
        }
    }

    pub fn as_pose_error(&self) -> &PoseErrorMetrics {
        match self {
            MetricType::PoseError(p) => p,
            _ => {
                panic!("wtf");
            }
        }
    }
}
pub trait MetricTypeInfo {
    fn type_name(&self) -> &'static str;
    fn as_any(&self) -> &dyn std::any::Any;
}


#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Copy)]
pub struct StatisticalMetricsStamped{
    pub stat: StatisticalMetrics,
    pub timestamp: f32,
}

impl Eq for StatisticalMetricsStamped {}

impl PartialOrd for StatisticalMetricsStamped {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.timestamp.partial_cmp(&other.timestamp)
    }
}

impl Ord for StatisticalMetricsStamped {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Use partial_cmp and unwrap. Assumes no NaN timestamps.
        self.partial_cmp(other).unwrap()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Copy)]
pub struct StatisticalMetrics {
    pub mean: f32,
    pub median: f32,
    pub min: f32,
    pub max: f32,
    pub std: f32,
    pub rmse: Option<f32>,    // Only for pose errors
    pub sse: Option<f32>,     // Only for pose errors
}


impl StatisticalMetrics{
    // Helper to compute mean of StatisticalMetrics
    pub fn mean(stats_metrics: &[&Self]) -> Option<StatisticalMetrics> {

        let count = stats_metrics.len() as f32;
        Some(StatisticalMetrics {
            mean: stats_metrics.iter().map(|s| s.mean).sum::<f32>() / count,
            median: stats_metrics.iter().map(|s| s.median).sum::<f32>() / count,
            min: stats_metrics.iter().map(|s| s.min).sum::<f32>() / count,
            max: stats_metrics.iter().map(|s| s.max).sum::<f32>() / count,
            std: stats_metrics.iter().map(|s| s.std).sum::<f32>() / count,
            rmse: Some(stats_metrics.iter().filter_map(|s| s.rmse).sum::<f32>() / count),
            sse: Some(stats_metrics.iter().filter_map(|s| s.sse).sum::<f32>() / count),
        })
    }

    pub fn from_values(values: &Vec<f32>, compute_rmse: bool) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
    
        // Clone and sort for median/min/max calculations
        let mut values_sorted = values.to_vec();
        values_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    
        // Calculate mean
        let mean = values_sorted.iter().sum::<f32>() / values_sorted.len() as f32;
    
        // Calculate median
        let median = if values_sorted.len() % 2 == 0 {
            let mid = values_sorted.len() / 2;
            (values_sorted[mid - 1] + values_sorted[mid]) / 2.0
        } else {
            values_sorted[values_sorted.len() / 2]
        };
    
        // Calculate standard deviation
        let variance = values_sorted.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>() / values_sorted.len() as f32;
        let std = variance.sqrt();
    
        // Calculate RMSE and SSE

        let (sse, rmse) = match compute_rmse{
            true => {
                let s = values_sorted.iter().map(|x| x.powi(2)).sum::<f32>();
                let r = (s / values_sorted.len() as f32).sqrt();
                (Some(s), Some(r))
            },
            false => (None, None)
        };
            
        Some(Self {
            mean,
            median,
            min: values_sorted[0],
            max: *values_sorted.last().unwrap(),
            std,
            rmse: rmse,
            sse: sse,
        })
    }


    pub fn from_single_value(value: f32) -> Self {
        Self {
            mean: value,
            median: value,
            min: value,
            max: value,
            std: 0.0,  // Standard deviation is undefined for single values, set to 0.0
            rmse: None,
            sse: None,
        }
    }


}


impl FromStr for StatisticalMetrics {
    type Err = services::error::EvoError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {

        let mut hash: HashMap<&str, f32> = HashMap::new();

        let _: Vec<_> = s.split("\n")
            .filter(|&s| s.contains("\t"))
            .map(|s| s.split("\t"))
            .flatten()
            .map(|s| s.trim())
            .batching( |it| {
                match it.next() {
                    None => None,
                    Some(x) => match it.next() {
                        None => None,
                        Some(y) => Some((x, y)),
                    }
                }
            }
            )
            .map(|(k, v)| {
                let n = v.parse::<f32>().unwrap();
                    hash.insert(k, n);
            }
        )
        .collect();

        return Ok(StatisticalMetrics{
            max: hash["max"],
            median: hash["median"],
            mean: hash["mean"],
            min: hash["min"],
            rmse: Some(hash["rmse"]),
            sse: Some(hash["sse"]),
            std: hash["std"]
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_statistical_metrics(val: f32) -> StatisticalMetrics {
        StatisticalMetrics {
            mean: val,
            median: val,
            min: val,
            max: val,
            std: 0.0,
            rmse: Some(val * 2.0),
            sse: Some(val * 3.0),
        }
    }

    #[test]
    fn test_statistical_metrics_from_values_basic() {
        let values = vec![1.0, 2.0, 3.0];
        let metrics = StatisticalMetrics::from_values(&values, true).unwrap();

        assert_eq!(metrics.mean, 2.0);
        assert_eq!(metrics.median, 2.0);
        assert_eq!(metrics.min, 1.0);
        assert_eq!(metrics.max, 3.0);
        assert!(metrics.std > 0.0);
        assert!(metrics.rmse.is_some());
        assert!(metrics.sse.is_some());
    }

    #[test]
    fn test_statistical_metrics_mean() {
        let a = mock_statistical_metrics(1.0);
        let b = mock_statistical_metrics(3.0);

        let mean = StatisticalMetrics::mean(&[&a, &b]).unwrap();

        assert_eq!(mean.mean, 2.0);
        assert_eq!(mean.median, 2.0);
        assert_eq!(mean.min, 2.0);
        assert_eq!(mean.max, 2.0);
        assert_eq!(mean.std, 0.0);
        assert_eq!(mean.rmse.unwrap(), 4.0);  // (2 + 6) / 2
        assert_eq!(mean.sse.unwrap(), 6.0);   // (3 + 9) / 2
    }

    #[test]
    fn test_statistical_metrics_from_str() {
        let input = "mean\t1.0\nmedian\t2.0\nmin\t0.5\nmax\t3.0\nstd\t0.5\nrmse\t0.8\nsse\t1.2\n";
        let parsed = input.parse::<StatisticalMetrics>().unwrap();

        assert_eq!(parsed.mean, 1.0);
        assert_eq!(parsed.median, 2.0);
        assert_eq!(parsed.min, 0.5);
        assert_eq!(parsed.max, 3.0);
        assert_eq!(parsed.std, 0.5);
        assert_eq!(parsed.rmse.unwrap(), 0.8);
        assert_eq!(parsed.sse.unwrap(), 1.2);
    }

    #[test]
    fn test_statistical_metrics_from_single_value() {
        let metric = StatisticalMetrics::from_single_value(5.0);
        assert_eq!(metric.mean, 5.0);
        assert_eq!(metric.std, 0.0);
        assert!(metric.rmse.is_none());
    }
}