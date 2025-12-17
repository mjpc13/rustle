use std::collections::HashMap;

use serde::Serialize;
use argmin::core::{ArgminFloat, Error};
use argmin::{float, argmin_error};
use rand_xoshiro::Xoshiro256PlusPlus;
use rand::{Rng, thread_rng};
use rand_distr::{Normal, Distribution, Uniform};

use serde_json::{Value, json, Map};

use crate::models::slam_config::SLAMConfig;

#[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
pub struct SimulatedAnnealingConfig {
    /// initial temperature
    pub initial_temp: f64,

    /// temperature function used to update the temperature value. needs the current temperature value and the current iteration
    pub temp_func: TemperatureFunction,

    /// number of iterations used for the calculation of the next temperature values. used for re-annealing(re-heating)
    temp_iter: u64,

    /// Number of iterations since the last accepted solution
    pub stall_iter_accepted: u64,

    /// Stop if `stall_iter_accepted` exceeds this number
    pub stall_iter_accepted_limit: u64,

    /// Number of iterations since the last best solution was found
    pub stall_iter_best: u64,

    /// Stop if `stall_iter_best` exceeds this number
    pub stall_iter_best_limit: u64,

    /// Reanneal after this number of iterations is reached
    reanneal_fixed: u64,

    /// Number of iterations since beginning or last reannealing
    reanneal_iter_fixed: u64,

    /// Reanneal after no accepted solution has been found for `reanneal_accepted` iterations
    reanneal_accepted: u64,

    /// Similar to `stall_iter_accepted`, but will be reset to 0 when reannealing  is performed
    reanneal_iter_accepted: u64,

    /// Reanneal after no new best solution has been found for `reanneal_best` iterations
    reanneal_best: u64,

    /// Similar to `stall_iter_best`, but will be reset to 0 when reannealing is performed
    reanneal_iter_best: u64,

    /// current temperature
    pub current_temp: f64,

    pub parameter_bounds: Option<HashMap<String, (Value, Value, Value)>>, // (starting value, lower bound, upper bound)

    pub max_iterations: Option<usize>,
}

impl SimulatedAnnealingConfig {
    pub fn new(initial_temperature: Option<f64>) -> Result<Self, Error> {
        match initial_temperature {
            Some(value) => {
                if value <= 0.0 {
                    return Err(argmin_error!(InvalidParameter, "`SimulatedAnnealing`: Initial temperature must be > 0."));
                }

                Ok(
                    SimulatedAnnealingConfig { 
                        initial_temp: value, 
                        temp_func: TemperatureFunction::Boltzman, 
                        temp_iter: 0, 
                        stall_iter_accepted: 0, 
                        stall_iter_accepted_limit: 10, // u64::MAX
                        stall_iter_best: 0, 
                        stall_iter_best_limit: 5, 
                        reanneal_fixed: 10, 
                        reanneal_iter_fixed: 0, 
                        reanneal_accepted: 10, 
                        reanneal_iter_accepted: 0, 
                        reanneal_best: 10, 
                        reanneal_iter_best: 0, 
                        current_temp: value, 
                        parameter_bounds: None, 
                        max_iterations: None}
                )     
            }
            None => {
                Ok(
                    SimulatedAnnealingConfig { 
                        initial_temp: 1.0, 
                        temp_func: TemperatureFunction::Boltzman, 
                        temp_iter: 0, 
                        stall_iter_accepted: 0, 
                        stall_iter_accepted_limit: 10, // u64::MAX
                        stall_iter_best: 0, 
                        stall_iter_best_limit: 5, 
                        reanneal_fixed: 10, 
                        reanneal_iter_fixed: 0, 
                        reanneal_accepted: 10, 
                        reanneal_iter_accepted: 0, 
                        reanneal_best: 10, 
                        reanneal_iter_best: 0, 
                        current_temp: 1.0, 
                        parameter_bounds: None, 
                        max_iterations: None}
                )     
            }
        }
    }

    pub fn update_temperature(&mut self) {
        self.current_temp = match self.temp_func {
            TemperatureFunction::TemperatureFast => {
                self.initial_temp / ((self.temp_iter + 1) as f64)
            }
            TemperatureFunction::Boltzman => {
                self.initial_temp / ((self.temp_iter + 1) as f64).ln()
            }
            TemperatureFunction::Exponential => {
                self.initial_temp * (0.95 as f64).powf((self.temp_iter + 1) as f64)
            }
        }
    }

    pub fn update_variables(&mut self) {
        self.temp_iter += 1;
    }

    pub fn update_slam_parameters(&self, current_params: &mut HashMap<String, Value>, params_bounds: &Option<HashMap<String, (Value, Value, Value)>>) {
        if let Some(bounds_map) = params_bounds {
            for (key, values) in bounds_map.iter() {
                //let bounds: (Value, Value, Value) = (value.as_array().unwrap()[0].clone(), value.as_array().unwrap()[1].clone(), value.as_array().unwrap()[2].clone());
                let bounds: (Value, Value, Value) = (values.0.clone(), values.1.clone(), values.2.clone());

                if bounds.0.is_f64() {
                    let old_value = current_params.get(key).unwrap().as_f64().unwrap();
                    let new_value: f64 = gaussian_perturbation(old_value, 0.5, (bounds.1.as_f64().unwrap(), bounds.2.as_f64().unwrap()));
                    current_params.insert(key.clone(), Value::from(new_value));

                    //println!("{} -> {}", old_value, new_value);
                }
                else if bounds.0.is_u64() {
                    let old_value: u64 = current_params.get(key).unwrap().as_u64().unwrap();
                    let new_value: u64 = integer_perturbation(old_value as i64, self.current_temp, 3, (bounds.1.as_u64().unwrap() as i64, bounds.2.as_u64().unwrap() as i64)) as u64;
                    current_params.insert(key.clone(), Value::from(new_value));
                }
                else if bounds.0.is_i64() {
                    let old_value: i64 = current_params.get(key).unwrap().as_i64().unwrap();
                    let new_value: i64 = integer_perturbation(old_value, self.current_temp, 3, (bounds.1.as_i64().unwrap(), bounds.2.as_i64().unwrap()));
                    current_params.insert(key.clone(), Value::from(new_value));
                }
            }
        }
    }

    /// if parameteters are specified in `parameter_bounds`, adjust their initial values
    pub fn get_initial_config(&self, params: &HashMap<String, Value>) -> HashMap<String, Value> {
        let mut initial_params = params.clone();

        if let Some(bounds_map) = self.parameter_bounds.clone() {
            for (key, values) in bounds_map.iter() {
                initial_params.insert(key.clone(), values.0.clone());
            }   
        }

        initial_params
    }

    pub fn accept_new_solution(&self, previous_iteration: &(f32, f32), current_iteration: &(f32, f32), metrics_weights: &(f32, f32)) -> bool {
        let previous_fitness = previous_iteration.0 * metrics_weights.0 + previous_iteration.1 * metrics_weights.1;
        let current_fitness = current_iteration.0 * metrics_weights.0 + current_iteration.1 * metrics_weights.1;
        let delta_solutions = current_fitness - previous_fitness;

        if delta_solutions < 0.0 {
            true
        }
        else {
            let mut rng = thread_rng();
            let niu = rng.gen();
            let p = (((-1 as f64) * (delta_solutions as f64)) / self.current_temp).exp();

            p > niu  
        }
    }
}

#[derive(Clone, PartialEq, Debug, serde::Serialize, serde::Deserialize)]
pub enum TemperatureFunction {
    TemperatureFast,

    Boltzman,

    Exponential,
}

pub fn cost_function(x: &f64, y: &f64) -> f64 {
    x * x + y * y
}

pub fn gaussian_perturbation(x: f64, sigma: f64, bounds: (f64, f64)) -> f64 {
    let normal = Normal::new(0.0, sigma).unwrap();
    let mut rng = thread_rng();

    (x + normal.sample(&mut rng)).clamp(bounds.0, bounds.1)
}

pub fn integer_perturbation(x: i64, temperature: f64, delta_max: i64, bounds: (i64, i64)) -> i64 {
    let max_step = (temperature * (delta_max as f64)).ceil() as i64;

    if max_step == 0 {
        return x;
    }

    let k = thread_rng().gen_range(-max_step..=max_step);
    (x + k).clamp(bounds.0, bounds.1)
}

/*
pub fn accept_new_solution(current_x: &f64, current_y: &f64, new_x: &f64, new_y: &f64, current_temperature: &f64) -> bool {
    let delta_solutions = cost_function(new_x, new_y) - cost_function(current_x, current_y);

    if delta_solutions < 0.0 {
        true
    }
    else {
        let mut rng = thread_rng();
        let niu = rng.gen();
        let p = (((-1 as f64) * delta_solutions) / current_temperature).exp();

        p > niu
    }
}
*/