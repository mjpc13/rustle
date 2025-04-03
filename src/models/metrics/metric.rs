use std::{collections::HashMap, str::FromStr};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use itertools::Itertools;
use serde_json::Value;

use crate::services::{self, DbError};
use surrealdb::sql::Thing;
use super::{cpu::CpuMetrics, pose_error::PoseErrorMetrics};


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metric {
    pub id: Option<Thing>,
    #[serde(rename = "metric_type")] // Matches SurrealDB's field name
    pub metric_type: MetricType
}


impl Metric {
    pub fn mean(metrics: Vec<Metric>) -> Vec<Metric> {

        let mut result: Vec<Metric> = Vec::new();

        //Vec of pose metrics
        let pose_metrics: Vec<&PoseErrorMetrics> = metrics.iter()
        .filter_map(|m| match &m.metric_type {
            MetricType::PoseError(p) => Some(p),
            _ => None
        })
        .collect();

        //Vec with only CPU metrics
        let cpu_metrics: Vec<&CpuMetrics> = metrics.iter()
        .filter_map(|m| match &m.metric_type {
            MetricType::Cpu(p) => Some(p),
            _ => None
        })
        .collect();

        //Add more possible future metrics here

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
        
        result
    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "PascalCase")]
pub enum MetricType{
    Cpu(CpuMetrics),
    PoseError(PoseErrorMetrics),
}

impl MetricType {
    pub fn type_name(&self) -> &'static str {
        match self {
            MetricType::Cpu(_) => "cpu",
            MetricType::PoseError(_) => "pose_error",
        }
    }
    
    pub fn as_any(&self) -> &dyn std::any::Any {
        match self {
            MetricType::Cpu(m) => m,
            MetricType::PoseError(m) => m,
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
    pub fn mean(stats_metrics: &[&Self]) -> StatisticalMetrics {

        let count = stats_metrics.len() as f64;
        StatisticalMetrics {
            mean: stats_metrics.iter().map(|s| s.mean).sum::<f64>() / count,
            median: stats_metrics.iter().map(|s| s.median).sum::<f64>() / count,
            min: stats_metrics.iter().map(|s| s.min).sum::<f64>() / count,
            max: stats_metrics.iter().map(|s| s.max).sum::<f64>() / count,
            std: stats_metrics.iter().map(|s| s.std).sum::<f64>() / count,
            rmse: Some(stats_metrics.iter().filter_map(|s| s.rmse).sum::<f64>() / count),
            sse: Some(stats_metrics.iter().filter_map(|s| s.sse).sum::<f64>() / count),
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