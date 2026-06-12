use std::{collections::HashMap, vec};
use serde_json::{Value, Number, json, Map};
use crate::{db::params, models::metrics::Metric};
use rand::RngExt;

use crate::models::tuning_config::find_key_in_hash_map;

// grid search can execute configurations concurrently
#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq)]
pub struct GridSearchConfig {
    pub configs: Vec<HashMap<String, Value>>,
    pub tunable_params: HashMap<String, Value>,
    
    /// for grid/random search
    pub tunable_params_array_sizes: Vec<(String, u64)>,
    
    /// for random search
    pub visited_points: Vec<Vec<(String, u64)>>,
}

impl GridSearchConfig {
    pub fn new() -> Self {
        Self { configs: Vec::new(), 
               tunable_params: HashMap::new(), 
               tunable_params_array_sizes: Vec::new(),
               visited_points: Vec::new(), }
    }

    pub fn build_array_sizes(&mut self) {
        for (key, value) in &self.tunable_params {
            self.tunable_params_array_sizes.push((key.clone(), value.clone().as_array().unwrap()[2].as_u64().unwrap()));
            //self.tunable_params_array_sizes.insert(key.clone(), value.clone().as_array().unwrap()[2].as_u64().unwrap());
        }
    }

    pub fn generate_random_point(&mut self) -> Option<Vec<(String, u64)>> {
        let mut random_point: Vec<(String, u64)> = Vec::new();
        let mut rng = rand::rng();

        let total_grid_size = self.get_param_array_size_by_key(None).unwrap();

        if self.visited_points.len() == self.get_param_array_size_by_index(None).unwrap() as usize {
            return None;
        }

        loop {
                        
            for (key, param_size) in &self.tunable_params_array_sizes {
                random_point.push((key.clone(), rng.random_range(0..param_size.clone())));
            }

            if !self.visited_points.contains(&random_point) {
                break;
            }
            else {
                random_point.clear();
            }
        }

        //self.add_visited_point(&random_point);
        self.visited_points.push(random_point.clone());
        return Some(random_point);
    }

    pub fn get_current_params(&self, grid_point: &Vec<(String, u64)>, initial_params: &HashMap<String, Value>) -> Option<HashMap<String, Value>> {
        //let tunable_params = self.tunable_params;
        //println!("{:?}", grid_point);
        let mut current_params = initial_params.clone();
        //println!("tunable params: {:?}", self.tunable_params);

        for (key, value) in &self.tunable_params {
            //match current_params.get(key).unwrap() {
            match find_key_in_hash_map(&mut current_params, key) {

                Some(Value::Number(n)) => {
                    if n.is_u64() {
                        let unsigned_int_tuple = self.u64_array_tuple(value);
                        //let next_value: u64 = unsigned_int_tuple.0 + grid_point.get(key).unwrap().clone() * unsigned_int_tuple.1;
                        let next_value: u64 = unsigned_int_tuple.0 + self.get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() * unsigned_int_tuple.1;
                        current_params.insert(key.clone(), Value::from(next_value));
                    }
                    else if n.is_i64() {
                        let int_tuple = self.i64_array_tuple(value);
                        //let next_value: i64 = (int_tuple.0) + (grid_point.get(key).unwrap().clone() as i64 * int_tuple.1);
                        let next_value: i64 = int_tuple.0 + (self.get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as i64) * int_tuple.1;
                        current_params.insert(key.clone(), Value::from(next_value));
                    }
                    else if n.is_f64() {
                        let float_tuple = self.f64_array_tuple(value);
                        //let next_value: f64 = float_tuple.0 + (grid_point.get(key).unwrap().clone() as f64) * float_tuple.1;
                        let next_value: f64 = float_tuple.0 + (self.get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as f64) * float_tuple.1;
                        current_params.insert(key.clone(), Value::from(next_value));
                    }
                }
                _ => return None
            }
        }
        return Some(current_params.clone());
    }

    /// if param is `Some()` then the number of values is provided for the specific parameter, if it exists.
    /// 
    /// if param is `None`, then the total number of configurations of the parameter space is provided
    pub fn get_param_array_size_by_key(&self, param: Option<&String>) -> Option<u64> {
        if let Some(param_key) = param {
            if let Some((key, param_size)) = self.tunable_params_array_sizes.iter().find(|(key, _)| key == param_key) {
                return Some(param_size.clone());
            }
            else {
                return None;
            }
        }
        else {
            return Some(self.tunable_params_array_sizes.iter().map(|(_, v)| v.clone()).product());      
        }
    }

    pub fn get_param_array_size_by_index(&self, index: Option<usize>) -> Option<u64> {
        if let Some(index_value) = index {
            if index_value >= self.tunable_params_array_sizes.len() {
                return None;
            }
            else {
                return Some(self.tunable_params_array_sizes[index_value].1.clone());
            }
        }
        else {
            return Some(self.tunable_params_array_sizes.iter().map(|(_, v)| v.clone()).product());  
        }
    }

    pub fn add_visited_point(&mut self, new_point: &Vec<(String, u64)>) {
        self.visited_points.push(new_point.clone());
    }

    pub fn get_grid_point_index(&self, grid_point: &Vec<(String, u64)>) -> Option<u64> {
        let mut total_index = 0;
        let mut multiplier = 1;

        for i in 0..grid_point.len() {
            total_index += grid_point[i].1 * multiplier;
            multiplier *= self.tunable_params_array_sizes[i].1.clone();
        }
        Some(total_index)

        /*
        let mut total_index: u64 = 0;
        for i in 0..grid_point.len() {
            total_index += grid_point[i].1 * self.tunable_params_array_sizes[i].1.pow(i as u32);
        }
        Some(total_index)
        */
    }

    fn get_parameter_array_current_index(&self, grid_point: Vec<(String, u64)>, target_key: String) -> Option<u64> {
        if let Some((key, index)) = grid_point.iter().find(|(key, _)| *key == target_key) {
            return Some(index.clone());
        }
        None
    }

    fn u64_array_tuple(&self, value: &Value) -> (u64, u64, u64) {
        let binding = value.clone();
        let array = binding.as_array().unwrap();
        let start: u64 = array[0].as_u64().unwrap();
        let step: u64 = array[1].as_u64().unwrap();
        let num_elements = array[2].as_u64().unwrap();

        (start, step, num_elements)
    }

    fn i64_array_tuple(&self, value: &Value) -> (i64, i64, u64) {
        let binding = value.clone();
        let array = binding.as_array().unwrap();
        let start: i64 = array[0].as_i64().unwrap();
        let step: i64 = array[1].as_i64().unwrap();
        let num_elements = array[2].as_u64().unwrap();

        (start, step, num_elements)
    }

    fn f64_array_tuple(&self, value: &Value) -> (f64, f64, u64) {
        let binding = value.clone();
        let array = binding.as_array().unwrap();
        let start: f64 = array[0].as_f64().unwrap();
        let step: f64 = array[1].as_f64().unwrap();
        let num_elements = array[2].as_u64().unwrap();

        (start, step, num_elements)
    }

    pub fn generate_initial_grid_point(&self) -> Option<Vec<(String, u64)>> {
        let mut first_point: Vec<(String, u64)> = Vec::new();

        for (key, value) in &self.tunable_params_array_sizes {
            first_point.push((key.clone(), 0));
        }

        Some(first_point)
    }

    pub fn generate_next_grid_point(&self, previous_point: Option<Vec<(String, u64)>>) -> Option<Vec<(String, u64)>> {
        if let None = previous_point {
            let mut first_point = self.tunable_params_array_sizes.clone();
            for (key, index) in &mut first_point {
                *index = 0;
            }
            return Some(first_point);
        }

        let mut next_point = previous_point.clone().unwrap();
        
        for i in 0..next_point.len() {
            next_point[i].1 += 1;

            // if no overflow, stop
            if next_point[i].1 < self.tunable_params_array_sizes[i].1 {
                break;
            }
            else {
                next_point[i].1 = 0;
            }

        }
        
        /*
        let first_element_size = &self.tunable_params_array_sizes[0].1.clone();

        if next_point[0].1.clone() == (first_element_size - 1) {
            next_point[0].1 = 0;
            for i in 1..next_point.len() {
                let current_key = next_point[i].0.clone();

                if next_point[i].1.clone() == self.get_param_array_size_by_key(Some(&current_key)).unwrap() - 1 {
                    next_point[0].1 = 0;
                }
                else {
                    next_point[1].1 += 1;
                    break;
                }
            }
            return Some(next_point);
        }
        */
        Some(next_point)
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
