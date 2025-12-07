use std::{collections::HashMap, vec};
use serde_json::{Value, Number, json, Map};
use crate::models::metrics::Metric;

// grid search can execute configurations concurrently
#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq)]
pub struct GridSearchConfig {
    pub configs: Vec<HashMap<String, Value>>,
    pub tunable_params: HashMap<String, Value>,
    
    /// for grid/random search
    pub tunable_params_array_sizes: HashMap<String, u64>,
    
    /// for random search
    pub visited_points: Vec<Vec<(String, u64)>>,
}

impl GridSearchConfig {
    pub fn new() -> Self {
        Self { configs: Vec::new(), 
               tunable_params: HashMap::new(), 
               tunable_params_array_sizes: HashMap::new(),
               visited_points: Vec::new(), }
    }

    pub fn build_configurations(&mut self, tunable_params: HashMap<String, Value>, gt_config: HashMap<String, Value>) {

        let mut tune_params = tunable_params.clone();

        let mut params_keys: Vec<String> = Vec::new();

        let mut initial_params = gt_config.clone();

        let mut keys_to_remove: Vec<String> = Vec::new();

        for (key, param) in &tune_params {
            if !param.is_array() {
                initial_params.insert(key.clone(), param.clone());
                keys_to_remove.push(key.clone());
            }
        }

        for k in keys_to_remove {
            tune_params.remove(&k);
        }

        let mut all_expanded_vecs: Vec<Vec<Value>> = Vec::new();

        // expand arrays
        for (key, value) in &tune_params {
            let arr = value.as_array().unwrap();
            if arr[0].is_u64() {
                all_expanded_vecs.push(self.unpack_u64_array(arr));
            }
            else if arr[0].is_f64() {
                all_expanded_vecs.push(self.unpack_f64_array(arr));
            }
            //all_expanded_vecs.push(self.unpack_f64_array(value.as_array().unwrap()));
        }
        //println!("");

        let combos = cartesian_product(&all_expanded_vecs);

        let total_configs= combos.len();
        let config_length= combos[0].len();

        // register each non constant parameter in params_iters
        for (key, param) in &tune_params {
            match param {
                Value::Array(_) => {
                    let arr_temp = param.as_array().unwrap();
                    params_keys.push(key.clone());
                },
                _ => {

                }
            }
        }

        let mut is_done: bool = false;

        loop {

            for i in 0..total_configs {
                for j in 0..config_length {
                    initial_params.insert(params_keys[j].clone(), combos[i][j].clone());
                }
                self.configs.push(initial_params.clone());
            }

            break;
        }

        /*
        for config in self.configs.clone() {
            println!("{:?} {:?} {:?}", config.get("acc_cov"), config.get("min_radius"), config.get("scan_resolution"));
        }
        */

    }

    // this function assumes arr is a json array(Value) of 3 elements [start, step, number_of_elements]
    fn unpack_f64_array(&self, arr: &Vec<Value>) -> Vec<Value> {
        let mut result_vec: Vec<Value> = Vec::new();
        let start = arr[0].as_f64().unwrap();
        let step = arr[1].as_f64().unwrap();
        let n: u64 = arr[2].as_u64().unwrap();

        for i in 0..n {
            //result_vec.push(json!(arr[0].as_f64().unwrap() + (i as f64) * arr[1].as_f64().unwrap()));
            result_vec.push(Value::from(start + (i as f64) * step));
        }

        result_vec
    }

    fn unpack_u64_array(&self, arr: &Vec<Value>) -> Vec<Value> {
        let mut result_vec: Vec<Value> = Vec::new();
        let start = arr[0].as_u64().unwrap();
        let step = arr[1].as_u64().unwrap();
        let n: u64 = arr[2].as_u64().unwrap();

        for i in 0..n {
            //result_vec.push(json!(arr[0].as_u64().unwrap() + (i as u64) * arr[1].as_u64().unwrap()));
            result_vec.push(Value::from(start + i * step));
        }

        result_vec
    }

    /// if param is `Some()` then the number of values is provided for the specific parameter, if it exists.
    /// 
    /// if param is `None`, then the total number of configurations of the parameter space is provided
    pub fn parameter_space_size(&self, param: Option<&String>) -> Option<usize> {
        if let Some(param) = param {
            if let Some((key, index)) = self.tunable_params_array_sizes.iter().find(|(key, _)| *key == param) {
                return Some(index.clone() as usize);
            }
            else {
                return None;
            }
        }
        else {
            let sum: u64 = self.tunable_params_array_sizes.iter().map(|(_, v)| v.clone()).product();
            return Some(sum as usize);
        }
    }

    pub fn build_array_sizes(&mut self) {
        for (key, value) in &self.tunable_params {
            self.tunable_params_array_sizes.insert(key.clone(), value.clone().as_array().unwrap()[2].as_u64().unwrap());
        }
    }


}

fn cartesian_product(data: &[Vec<Value>]) -> Vec<Vec<Value>> {
    let mut result = vec![vec![]];
    for vec in data {
        result = result
            .iter()
            .flat_map(|prefix| {
                vec.iter().map(move |v| {
                    let mut new = prefix.clone();
                    new.push(v.clone());
                    new
                })
            })
            .collect();
    }
    result
}