use crate::{errors::{EvoError, RosError}, evo_wrapper::EvoArg, ros_msgs::{GeometryMsg, Odometry, RosMsg}, task::{Config, TaskOutput}};
use serde::{Deserialize, Serialize};
use bollard::container::{MemoryStats, CPUStats};
use chrono::{DateTime, Utc};

use surrealdb::{
    Surreal,
    engine::any
};
use temp_dir::TempDir;
use std::{collections::HashMap, env, fs::OpenOptions, io::Write, path::PathBuf, str::FromStr};
use itertools::Itertools;

#[derive(Debug, Serialize, Deserialize)]
pub struct ContainerStats {
    pub uid: Option<u32>,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub precpu_stats: CPUStats,
    pub num_procs: u32,
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Metric{
    max: f32,
    median: f32,
    min: f32,
    rmse: f32,
    sse: f32,
    std: f32
}


impl Metric{
    pub fn compute<T: GeometryMsg, R: EvoArg>(data: TaskOutput<T>, args: R, output_path: Option<&str>) -> Result<Vec<Metric>, EvoError>{
        
        //Write TUM files to temporary directory
        let mut path = match output_path{
            Some(p) => {
                PathBuf::from(p)
            },
            None => {
                env::temp_dir()
            }
        };
        let mut path_gt = path.clone();
        
        Self::write_file::<T>(data.groundtruth, "groundtruth", &mut path_gt);

        let metric_results: Vec<_> = data.odoms
            .into_iter()
            .map( |(s, v)| {
                let mut path_clone = path.clone();

                Self::write_file(v, &s, &mut path_clone);
                let evo_str = args.compute(path_gt.to_str().unwrap(), path_clone.to_str().unwrap()).unwrap();
                Metric::from_str(&evo_str).unwrap()

            })
            .collect();

        Ok(metric_results)
        

        //println!("{}", &evo_str);
        //todo!();
        //Metric::from_str(&evo_str)
    }

    fn write_file<'a, T: GeometryMsg>(data: Vec<T>, name: &str, path: &mut PathBuf){

        path.push(name.replace("/", "-"));

        let mut file = OpenOptions::new()
            .write(true)
            .append(true)
            .create(true)
            .open(path).unwrap();
        
        let _: Vec<_> = data.iter()
            .map(|o|{
                if let Err(e) = writeln!(file, "{}", o ){
                    eprintln!("Couldn't write to file: {}", e);
                }
            })
            .collect();

    }
}

impl FromStr for Metric {
    type Err = EvoError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut hash: HashMap<&str, f32> = HashMap::new();

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

        return Ok(Metric{
            max: hash["max"],
            median: hash["median"],
            min: hash["min"],
            rmse: hash["rmse"],
            sse: hash["sse"],
            std: hash["std"]
        })
    }
}




pub struct ContainerMetrics{

}