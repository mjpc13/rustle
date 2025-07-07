use std::collections::HashMap;

use log::warn;
use serde::{Deserialize, Serialize};

use crate::{models::{metrics::PoseErrorMetrics, test_definitions::test_definition::RobustnessType}, services::error::MetricError};

use super::metric::StatisticalMetrics;
use surrealdb::sql::Thing;




pub struct RobustnessMetric {
    pub sev: f32,                       // Severety of the cut//drop.
    pub adn: f32,                      // Absolute Degradation;
    pub rdn: f32,                      // Relative Degradation;
    pub ri: f32,                       // Robustness Index;
    pub art: f32,                      // Absolute Recovery Time; Only has values if the algo has loop closure
    pub rrt: f32,                      // Relative Recovery Time;
}


impl RobustnessMetric{

    pub fn new(rob_type: RobustnessType, duration: f32) -> Result<Self, RobustnessMetric>{

        //Compute the severity of the cut//drop
        // 

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

        let pdn = compute_adn();


        todo!()
    }

}

fn compute_adn() -> Result<f32, MetricError>{
    
    //(Error Cut - Error baseline) / severity

    todo!()
}
