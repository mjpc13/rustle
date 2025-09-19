use std::collections::HashMap;
use serde_json::Value;
use std::fs::{File, self};
use std::io::{BufReader, Write, BufRead};

pub enum TuneType {
    GridSearch(GridSearchConfig),
}

pub struct GridSearchConfig {
    algo: String,
    dataset: String,
    param_values: HashMap<String, Vec<Value>>,
}

impl GridSearchConfig {
    pub fn new(algo_name: String, dataset_name: String, params_file: &str) -> Result<Self, Box<dyn std::error::Error>> {
        match Self::parse_yaml(params_file) {
            Ok(result) => Ok(Self {
                param_values: result,
                algo: algo_name,
                dataset: dataset_name,
        }),
            Err(e) => {
                Err(e)
            }
        }
    }

    fn parse_yaml(file_path: &str) -> Result<HashMap<String, Vec<Value>>, Box<dyn std::error::Error>> {
        let file = File::open(file_path)?;
        let reader = BufReader::new(file);

        let mut config: HashMap<String, Vec<Value>> = HashMap::new();
        config = serde_yaml::from_reader(reader)?;

        Ok(config)
    }
}

// if boolean, provide only maximum 2 values, true or false
// if int or unsigned int, provide start, stop and step, like this: [1, 10, 1] -> [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// if f32 or f64, same as int. example: [1.34, 1.39, 0.01] -> [1.34, 1.35, 1.36, 1.37, 1.38, 1.39]
// if String, provide all possible values, like this: ["/livox/lidar", "/velodyne_points"]
// if vector, provide all possible vectors, like this: [[0.04165, 0.02326, -0.0284], [0.96535, 32.6, -9.34653]]