use std::{collections::HashMap, str::FromStr};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use itertools::Itertools;

use crate::services;
use surrealdb::sql::Thing;
use super::{cpu::CpuMetrics, pose_error::PoseErrorMetrics};


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metric {
    pub id: Option<Thing>,
    pub metric_type: MetricType
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MetricType{
    Cpu(CpuMetrics),
    PoseError(PoseErrorMetrics),
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