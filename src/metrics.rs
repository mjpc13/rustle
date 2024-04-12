use crate::{db::{Stats, DB}, errors::{EvoError, RosError}};
use surrealdb::{
    Surreal,
    engine::any
};
use std::collections::HashMap;
use itertools::Itertools;

enum LocalizationMetric {
    APE(Metrics),
    RTE(Metrics)
}

#[derive(Debug)]
pub struct Metrics{
    max: f32,
    median: f32,
    min: f32,
    rmse: f32,
    sse: f32,
    std: f32
}

impl Metrics {

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
    
}
