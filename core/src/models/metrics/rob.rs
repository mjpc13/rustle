use std::collections::HashMap;

use log::warn;
use serde::{Deserialize, Serialize};

use crate::{models::{metrics::PoseErrorMetrics, test_definitions::test_definition::RobustnessType, AlgorithmRun}, services::error::MetricError};

use super::metric::StatisticalMetrics;
use surrealdb::sql::Thing;



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessMetric {
    pub sev: f32,                       // Severety of the cut//drop.
    pub adn: f32,                      // Absolute Degradation;
    pub rdn: f32,                      // Relative Degradation;
    pub ri: f32,                       // Robustness Index;
    pub art: f32,                      // Absolute Recovery Time; Only has values if the algo has loop closure
    pub rrt: f32,                      // Relative Recovery Time;
}


impl RobustnessMetric{

    pub fn new(rob_type: RobustnessType, duration: f32, test: &AlgorithmRun, baseline: &AlgorithmRun) -> Result<Self, RobustnessMetric>{

        let sev = match rob_type {
            RobustnessType::Cut(cut_vec) => {

                let mut durations: Vec<f32>= vec![];

                cut_vec.iter().for_each(|cut|{
                    cut.active_periods.iter().for_each(|ap|{
                        durations.push(ap.duration_sec as f32);
                    });
                });
                
                let cut_duration: f32 = durations.iter().sum::<f32>();
                cut_duration / duration
                
            },
            RobustnessType::Drop(drop_vec) => {

                let mut durations: Vec<f32>= vec![];
    
                drop_vec.iter().for_each(|drop|{
                    let d = drop.drop_rate[0] as f32/ drop.drop_rate[1] as f32;

                    drop.active_periods.iter().for_each(|ap|{
                        durations.push(ap.duration_sec as f32 * d);
                    });
                });
                
                let drop_duration: f32 = durations.iter().sum::<f32>();
                drop_duration / duration
            },
        };

        let adn = compute_adn(test, baseline, &sev);
        let rdn = compute_rdn(test, baseline, &sev);

        warn!("This is the adn: {:?}\n and this is the RDN: {:?}", adn, rdn);



        todo!()
    }

}

fn compute_adn(test: &AlgorithmRun, baseline: &AlgorithmRun, sev: &f32) -> Result<f32, MetricError>{
    
    let adn_list: Vec<f32> = test.ape_list
        .iter()
        .zip(baseline.ape_list.iter())
        .map(|(st_test, st_baseline)|{
            ((st_test.stat.mean - st_baseline.stat.mean).abs()) / sev
        }).collect();

        // Example final aggregation: mean of adn_list
    if adn_list.is_empty() {
        return Err(MetricError::MissingError("ADN list is empty".to_owned()));
    }

    let adn_mean = adn_list.iter().sum::<f32>() / adn_list.len() as f32;

    Ok(adn_mean)

}

fn compute_rdn(test: &AlgorithmRun, baseline: &AlgorithmRun, sev: &f32) -> Result<f32, MetricError>{
    
    let rdn_list: Vec<f32> = test.rpe_list
        .iter()
        .zip(baseline.rpe_list.iter())
        .map(|(st_test, st_baseline)|{
            ((st_test.stat.mean - st_baseline.stat.mean).abs()) / sev
        }).collect();

        // Example final aggregation: mean of adn_list
    if rdn_list.is_empty() {
        return Err(MetricError::MissingError("ADN list is empty".to_owned()));
    }

    let rdn_mean = rdn_list.iter().sum::<f32>() / rdn_list.len() as f32;

    Ok(rdn_mean)

}
