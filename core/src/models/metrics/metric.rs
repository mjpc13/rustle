use std::{collections::HashMap, str::FromStr};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use itertools::Itertools;
use serde_json::Value;

use crate::services::{self, DbError};
use surrealdb::sql::Thing;
use super::{cpu::CpuMetrics, memory::MemoryMetrics, pose_error::PoseErrorMetrics};



pub struct AggregatedMetric{
    //
}











#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metric {
    pub id: Option<Thing>,
    #[serde(rename = "metric_type")] // Matches SurrealDB's field name
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

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "PascalCase")]
pub enum MetricType{
    Cpu(CpuMetrics),
    Memory(MemoryMetrics),
    PoseError(PoseErrorMetrics),
    Frequency(StatisticalMetrics)
}

impl MetricType {
    pub fn type_name(&self) -> &'static str {
        match self {
            MetricType::Cpu(_) => "cpu",
            MetricType::PoseError(_) => "pose_error",
            MetricType::Frequency(_) => "frequency",
            MetricType::Memory(_) => "memory"
        }
    }
    
    pub fn as_any(&self) -> &dyn std::any::Any {
        match self {
            MetricType::Cpu(m) => m,
            MetricType::PoseError(m) => m,
            MetricType::Frequency(m) => m,
            MetricType::Memory(m) => m
        }
    }
}
pub trait MetricTypeInfo {
    fn type_name(&self) -> &'static str;
    fn as_any(&self) -> &dyn std::any::Any;
}



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatisticalMetrics {
    pub mean: f64,
    pub median: f64,
    pub min: f64,
    pub max: f64,
    pub std: f64,
    pub rmse: Option<f64>,    // Only for pose errors
    pub sse: Option<f64>,     // Only for pose errors
}


impl StatisticalMetrics{
    // Helper to compute mean of StatisticalMetrics
    pub fn mean(stats_metrics: &[&Self]) -> Option<StatisticalMetrics> {

        let count = stats_metrics.len() as f64;
        Some(StatisticalMetrics {
            mean: stats_metrics.iter().map(|s| s.mean).sum::<f64>() / count,
            median: stats_metrics.iter().map(|s| s.median).sum::<f64>() / count,
            min: stats_metrics.iter().map(|s| s.min).sum::<f64>() / count,
            max: stats_metrics.iter().map(|s| s.max).sum::<f64>() / count,
            std: stats_metrics.iter().map(|s| s.std).sum::<f64>() / count,
            rmse: Some(stats_metrics.iter().filter_map(|s| s.rmse).sum::<f64>() / count),
            sse: Some(stats_metrics.iter().filter_map(|s| s.sse).sum::<f64>() / count),
        })
    }

    pub fn from_values(values: &Vec<f64>, compute_rmse: bool) -> Option<Self> {
        if values.is_empty() {
            return None;
        }
    
        // Clone and sort for median/min/max calculations
        let mut values_sorted = values.to_vec();
        values_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    
        // Calculate mean
        let mean = values_sorted.iter().sum::<f64>() / values_sorted.len() as f64;
    
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
            .sum::<f64>() / values_sorted.len() as f64;
        let std = variance.sqrt();
    
        // Calculate RMSE and SSE

        let (sse, rmse) = match compute_rmse{
            true => {
                let s = values_sorted.iter().map(|x| x.powi(2)).sum::<f64>();
                let r = (s / values_sorted.len() as f64).sqrt();
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


    pub fn from_single_value(value: f64) -> Self {
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

        let mut hash: HashMap<&str, f64> = HashMap::new();

        let data_vec: Vec<_> = s.split("\n")
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
                let n = v.parse::<f64>().unwrap();
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