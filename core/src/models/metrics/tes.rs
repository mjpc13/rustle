use std::collections::HashMap;

use log::warn;
use serde::{Deserialize, Serialize};

use crate::{models::metrics::PoseErrorMetrics, services::error::MetricError};

use super::metric::StatisticalMetrics;
use surrealdb::sql::Thing;




#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
pub struct TemporalEfficiencyMetric {
    pub speed_freq: HashMap<String, StatisticalMetrics>,    // HashMap speed multiplier to frequency of estimates
    pub eff_s: HashMap<String, f32>,        // Hashmap for the ratio between freq. at 1x and freq at sx; (Greater the value the worse the performance)
    pub tes: f32,                       // Temporal Efficiency Score (TES) OR use a Absolute Temporal Efficiency Score AND Relative Temporal Efficiency Score
    pub fpt: f32,                      // Frame Processing Time (FPT)
    pub atas: f32,                    // Absolute Time Accuracy Score, each eff_s multiply by the RMSE APE value;
    pub rtas: f32,                   // Relative Time Accuracy Score, each eff_s multiply by the RMSE RPE value;
}


impl TemporalEfficiencyMetric{

    pub fn new(speed_freq: HashMap<String, StatisticalMetrics>, speed_pose: HashMap<String, PoseErrorMetrics>) -> Result<Self, MetricError>{

        let mut eff_s: HashMap<String, f32> = HashMap::new();

        let base_freq = speed_freq
            .get("1")
            .ok_or(MetricError::ComputeError("Missing 1x speed in speed_freq".to_owned()))?
            .mean;

        
        let _ = speed_freq.iter()
            .for_each(|(s, f)| {
                eff_s.insert(s.to_string(), base_freq/f.mean);
            });

        let tes_accum: f32 = eff_s.values().sum();
        let list_size:f32 = eff_s.values().len() as f32;

        let tes = if eff_s.is_empty() {
                0.0
            } else {
                tes_accum / list_size
            };


        let fpt_value_opt = eff_s.clone().into_values()
            .filter(|&x| (1.0 - x).abs() > 0.1)// Filter out values at 90% of initial freq
            .min_by(|a, b| a.partial_cmp(b).unwrap());

        let fpt_time = if let Some(fpt_value) = fpt_value_opt {
            let fpt_freq = eff_s.iter()
                .find(|(_, &v)| v == fpt_value)
                .map(|(k, _)| k.parse::<f32>().unwrap())
                .ok_or(MetricError::ComputeError("Could not compute Frame Processing Time".to_owned()))?;
        
            1.0 / (fpt_freq * fpt_value)
        } else {
            warn!("Algorithm estimation frequency did not drop by 10% for the given set. Increase maximum bag speed value.");
            -1.0
        };

        let tas = compute_tas(&eff_s, speed_pose);

        Ok(TemporalEfficiencyMetric{
            speed_freq,
            eff_s,
            tes,
            fpt: fpt_time,
            atas: tas[0],
            rtas: tas[1]
        })

    }

}

fn compute_tas(eff_s: &HashMap<String,f32>, speed_pose: HashMap<String, PoseErrorMetrics>) -> [f32; 2]{

    let mut atas_list: Vec<f32> = Vec::new();
    let mut rtas_list: Vec<f32> = Vec::new();

    for (k, eff_val) in eff_s.clone() {
        if let Some(pose) = speed_pose.get(&k) {
            atas_list.push(eff_val * pose.ape.mean);
            rtas_list.push(eff_val * pose.rpe.mean);
        }
    }

    let list_size: f32 = atas_list.len() as f32;

    let atas_accum: f32 = atas_list.into_iter().sum();
    let rtas_accum: f32 = rtas_list.into_iter().sum();

    [atas_accum/list_size, rtas_accum/list_size]

}