use crate::{errors::{EvoError, RosError}, evo_wrapper::{evo_ape, EvoArgs}};
use serde::{Deserialize, Serialize};
use bollard::container::{MemoryStats, CPUStats};
use chrono::{DateTime, Utc};

use surrealdb::{
    Surreal,
    engine::any
};
use std::collections::HashMap;
use itertools::Itertools;


#[derive(Debug, Serialize, Deserialize)]
pub struct Stats {
    pub uid: Option<u32>,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub num_procs: u32,
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Metrics{
    max: f32,
    median: f32,
    min: f32,
    rmse: f32,
    sse: f32,
    std: f32
}



impl Metrics{

    pub fn parse(data: &str) -> Result<Metrics, EvoError>{

        let mut hash: HashMap<&str, f32> = HashMap::new();

        let data_vec: Vec<_> = data.split("\n")
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
        })
        .map(|(k, v)| {
            let n = v.parse::<f32>().unwrap();
                //.unwrap_or(
                //return Err::<(),RosError>(
                //        RosError::ParseError { 
                //            name: "asd".into(), 
                //            value: "dsa".into() 
                //        }
                //    )
                //);
                hash.insert(k, n);
        }
        )
        .collect();

        return Ok(Metrics{
            max: hash["max"],
            median: hash["median"],
            min: hash["min"],
            rmse: hash["rmse"],
            sse: hash["sse"],
            std: hash["std"]
        })
    }
    
    pub fn compute_ape(&self, groundtruth: &str, data: &str, args: EvoArgs) -> Metrics {

        evo_ape(groundtruth, data, args).unwrap()

    }

}
