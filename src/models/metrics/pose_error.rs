use std::{fs::File, io::{self, BufRead, BufReader}};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::services::error::MetricError;

use super::metric::{MetricTypeInfo, StatisticalMetrics};
use surrealdb::sql::Thing;



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct APE{
    pub id: Option<Thing>,
    pub value: f64,
    pub time_from_start: f64
}

impl APE {
    pub fn read_from_file(path: &str) -> Result<Vec<APE>, MetricError> {
        let file = File::open(path)
            .map_err(|e| MetricError::IOError(format!("Failed to open {}: {}", path, e)))?;

        BufReader::new(file)
            .lines()
            .enumerate()
            .map(|(line_num, line)| {
                let line = line.map_err(|e| MetricError::IOError(format!("Line {}: {}", line_num + 1, e)))?;
                
                let mut parts = line.split(',');
                let time = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing time value", line_num + 1)))?;
                let value = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing APE value", line_num + 1)))?;

                // Check for extra fields
                if parts.next().is_some() {
                    return Err(MetricError::ParseError(format!("Line {}: Too many fields", line_num + 1)));
                }

                Ok(APE {
                    id: None,
                    time_from_start: time.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid time format: {}", line_num + 1, e)))?,
                    value: value.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid APE value: {}", line_num + 1, e)))?,
                })
            })
            .collect()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RPE{
    pub id: Option<Thing>,
    pub value: f64,
    pub time_from_start: f64
}
impl RPE {
    pub fn read_from_file(path: &str) -> Result<Vec<RPE>, MetricError> {
        let file = File::open(path)
            .map_err(|e| MetricError::IOError(format!("Failed to open {}: {}", path, e)))?;

        BufReader::new(file)
            .lines()
            .enumerate()
            .map(|(line_num, line)| {
                let line = line.map_err(|e| MetricError::IOError(format!("Line {}: {}", line_num + 1, e)))?;
                
                let mut parts = line.split(',');
                let time = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing time value", line_num + 1)))?;
                let value = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing RPE value", line_num + 1)))?;

                // Validate no extra columns
                if parts.next().is_some() {
                    return Err(MetricError::ParseError(format!("Line {}: Too many fields", line_num + 1)));
                }

                Ok(RPE {
                    id: None,
                    time_from_start: time.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid time format: {}", line_num + 1, e)))?,
                    value: value.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid RPE value: {}", line_num + 1, e)))?,
                })
            })
            .collect()
    }
}



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position{
    pub id: Option<Thing>,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub time_from_start: f64
}
impl Position {
    pub fn read_from_file(path: &str) -> Result<Vec<Position>, MetricError> {
        let file = File::open(path)
            .map_err(|e| MetricError::IOError(format!("Failed to open {}: {}", path, e)))?;

        BufReader::new(file)
            .lines()
            .enumerate()
            .map(|(line_num, line)| {
                let line = line.map_err(|e| MetricError::IOError(format!("Line {}: {}", line_num + 1, e)))?;
                
                let mut parts = line.split(',');
                let time = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing time value", line_num + 1)))?;
                let x = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing x coordinate", line_num + 1)))?;
                let y = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing y coordinate", line_num + 1)))?;
                let z = parts.next()
                    .ok_or_else(|| MetricError::ParseError(format!("Line {}: Missing z coordinate", line_num + 1)))?;

                // Validate exact 4 columns
                if parts.next().is_some() {
                    return Err(MetricError::ParseError(format!("Line {}: Too many fields", line_num + 1)));
                }

                Ok(Position {
                    id: None,
                    time_from_start: time.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid time format: {}", line_num + 1, e)))?,
                    x: x.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid x format: {}", line_num + 1, e)))?,
                    y: y.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid y format: {}", line_num + 1, e)))?,
                    z: z.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Line {}: Invalid z format: {}", line_num + 1, e)))?,
                })
            })
            .collect()
    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseErrorMetrics {
    pub ape: StatisticalMetrics,
    pub rpe: StatisticalMetrics,
    #[serde(rename = "created_at")] // Match SurrealDB's field name
    pub created_at: DateTime<Utc>
}

impl MetricTypeInfo for PoseErrorMetrics {
    fn type_name(&self) -> &'static str { "pose_error" }
    fn as_any(&self) -> &dyn std::any::Any { self }
}


impl PoseErrorMetrics {
    pub fn mean(metrics: &[&Self]) -> Option<PoseErrorMetrics> {
        if metrics.is_empty() {
            return None;
        }
    
        let created_at = metrics.iter()
            .map(|m| m.created_at)
            .max()
            .unwrap_or_else(Utc::now);
    
    
        Some(PoseErrorMetrics {
            ape: StatisticalMetrics::mean(&metrics.iter().map(|m| &m.ape).collect::<Vec<_>>()).ok_or(MetricError::ComputeError("Could not compute mean of ape".to_owned())).unwrap(),
            rpe: StatisticalMetrics::mean(&metrics.iter().map(|m| &m.rpe).collect::<Vec<_>>()).ok_or(MetricError::ComputeError("Could not compute mean of rpe".to_owned())).unwrap(),
            created_at,
        })
    }

    pub fn from_values(ape_values: &Vec<f64>, rpe_values: &Vec<f64>) -> Result<PoseErrorMetrics, MetricError> {

        let ape = StatisticalMetrics::from_values(ape_values, true).ok_or(MetricError::ComputeError("Missing APE".to_owned()))?;
        let rpe = StatisticalMetrics::from_values(rpe_values, true).ok_or(MetricError::ComputeError("Missing RPE".to_owned()))?;

        Ok(Self { ape, rpe, created_at: Utc::now() })


    }
}

// Helper function to read metric files
fn read_metric_file(path: &str) -> Result<Vec<f64>, MetricError> {
    let file = File::open(path)
        .map_err(|e| MetricError::IOError(format!("Failed to open {}: {}", path, e)))?;
        
    BufReader::new(file)
        .lines()
        .map(|line| {
            line.map_err(|e| MetricError::IOError(format!("Read error: {}", e)))
                .and_then(|s| {
                    s.trim().parse()
                        .map_err(|e| MetricError::ParseError(format!("Failed to parse value: {}", e)))
                })
        })
        .collect()
}