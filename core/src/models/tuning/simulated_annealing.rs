use std::collections::HashMap;

use serde::Serialize;
use argmin::core::{ArgminFloat, Error};
use argmin::{float, argmin_error};
use rand_xoshiro::Xoshiro256PlusPlus;
use rand::{Rng, thread_rng};
use rand_distr::{Normal, Distribution, Uniform};

use serde_json::{Value, json, Map};

use crate::models::slam_config::SLAMConfig;
use crate::models::tuning_config::find_key_in_hash_map;

#[derive(Debug, PartialEq, Clone, serde::Serialize, serde::Deserialize)]
pub struct SimulatedAnnealingConfig {
    /// initial temperature
    pub initial_temp: f64,

    /// temperature function used to update the temperature value. needs the current temperature value and the current iteration
    pub temp_func: TemperatureFunction,

    /// number of iterations used for the calculation of the next temperature values. used for re-annealing(re-heating)
    pub temp_iter: u64,

    /// Number of iterations since the last accepted solution
    pub stall_iter_accepted: u64,

    /// Stop if `stall_iter_accepted` exceeds this number
    pub stall_iter_accepted_limit: u64,

    /// Number of iterations since the last best solution was found
    pub stall_iter_best: u64,

    /// Stop if `stall_iter_best` exceeds this number
    pub stall_iter_best_limit: u64,

    /// Reanneal after this number of iterations is reached
    pub reanneal_fixed: u64,

    /// Number of iterations since beginning or last reannealing
    pub reanneal_iter_fixed: u64,

    /// Reanneal after no accepted solution has been found for `reanneal_accepted` iterations
    pub reanneal_accepted: u64,

    /// Similar to `stall_iter_accepted`, but will be reset to 0 when reannealing  is performed
    pub reanneal_iter_accepted: u64,

    /// Reanneal after no new best solution has been found for `reanneal_best` iterations
    pub reanneal_best: u64,

    /// Similar to `stall_iter_best`, but will be reset to 0 when reannealing is performed
    pub reanneal_iter_best: u64,

    /// current temperature
    pub current_temp: f64,

    pub parameter_bounds: Option<HashMap<String, (Value, Value, Value, Value, Option<Value>)>>, // (starting value, lower bound, upper bound)

    pub max_iterations: Option<usize>,

    pub integer_delta_max: i64,
    pub float_mean: f64,
    pub float_std_dev: f64,
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
                        stall_iter_accepted_limit: 10,
                        stall_iter_best: 0, 
                        stall_iter_best_limit: 10, 
                        reanneal_fixed: 6, 
                        reanneal_iter_fixed: 0, 
                        reanneal_accepted: 5, 
                        reanneal_iter_accepted: 0, 
                        reanneal_best: 5, 
                        reanneal_iter_best: 0, 
                        current_temp: value, 
                        parameter_bounds: None, 
                        max_iterations: None,
                        integer_delta_max: 3,
                        float_mean: -0.5,
                        float_std_dev: 0.4}
                )     
            }
            None => {
                Ok(
                    SimulatedAnnealingConfig { 
                        initial_temp: 1.0, 
                        temp_func: TemperatureFunction::Boltzman, 
                        temp_iter: 0, 
                        stall_iter_accepted: 0, 
                        stall_iter_accepted_limit: 10,
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
                        max_iterations: None,
                        integer_delta_max: 3,
                        float_mean: -0.5,
                        float_std_dev: 0.4}
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

    pub fn update_slam_parameters(&self, current_params: &mut HashMap<String, Value>, params_bounds: &Option<HashMap<String, (Value, Value, Value, Value, Option<Value>)>>) {
        if let Some(bounds_map) = params_bounds {
            for (key, values) in bounds_map.iter() {
                let bounds: (Value, Value, Value, Value, Option<Value>) = (values.0.clone(), values.1.clone(), values.2.clone(), values.3.clone(), values.4.clone());

                if bounds.0.is_f64() {
                    let mut current_value = find_key_in_hash_map(current_params, key).unwrap();
                    let new_value: f64 = self.gaussian_perturbation(current_value.clone().as_f64().unwrap(), (bounds.1.as_f64().unwrap(), bounds.2.as_f64().unwrap()), 
                                                                                bounds.3.as_f64().unwrap(), bounds.4.unwrap().as_f64().unwrap());

                    *current_value = Value::from(new_value);
                }
                else if bounds.0.is_u64() {
                    let mut current_value = find_key_in_hash_map(current_params, key).unwrap();
                    let new_value: u64 = self.integer_perturbation(current_value.as_u64().unwrap() as i64, (bounds.1.as_u64().unwrap() as i64, bounds.2.as_u64().unwrap() as i64), 
                                                                   bounds.3.as_u64().unwrap() as i64) as u64;

                    *current_value = Value::from(new_value);
                }
                else if bounds.0.is_i64() {
                    let mut current_value = find_key_in_hash_map(current_params, key).unwrap();
                    let new_value: i64 = self.integer_perturbation(current_value.as_i64().unwrap(), (bounds.1.as_i64().unwrap(), bounds.2.as_i64().unwrap()), 
                                                                   bounds.3.as_u64().unwrap() as i64);

                    *current_value = Value::from(new_value);
                }
            }
        }
    }

    pub fn reanneal(&mut self, i: usize, iter_last_reset: &mut usize) {
        let out = (
            self.reanneal_iter_fixed >= self.reanneal_fixed,
            self.reanneal_iter_accepted >= self.reanneal_accepted,
            self.reanneal_iter_best >= self.reanneal_best,
        );
        if out.0 || out.1 || out.2 {
            self.reanneal_iter_fixed = 0;
            self.reanneal_iter_accepted = 0;
            self.reanneal_iter_best = 0;
            // reset temperature and temperature function iterations variable
            self.current_temp = self.initial_temp;
            self.temp_iter = 0;

            // new stuff
            *iter_last_reset = i;
        }
    }

    pub fn get_initial_config(&self, params: &HashMap<String, Value>) -> HashMap<String, Value> {
        let mut initial_params = params.clone();

        if let Some(bounds_map) = self.parameter_bounds.clone() {
            for (key, values) in bounds_map.iter() {
                let mut new_param = find_key_in_hash_map(&mut initial_params, key).unwrap();
                *new_param = Value::from(values.0.clone());
            }   
        }

        initial_params
    }

    pub fn gaussian_perturbation(&self, x: f64, bounds: (f64, f64), mean: f64, std_dev: f64) -> f64 {
        //let normal = Normal::new(self.float_mean, self.float_std_dev).unwrap();
        let normal = Normal::new(mean, std_dev).unwrap();
        let mut rng = thread_rng();

        (x + normal.sample(&mut rng)).clamp(bounds.0, bounds.1)
    }

    pub fn integer_perturbation(&self, x: i64, bounds: (i64, i64), delta_max: i64) -> i64 {
        let max_step = (self.current_temp * (delta_max as f64)).ceil() as i64;

        if max_step == 0 {
            return x;
        }

        let k = thread_rng().gen_range(-max_step..=max_step);
        (x + k).clamp(bounds.0, bounds.1)
    }

    pub fn accept_new_solution(&self, current_fitness: &f32, proposed_fitness: &f32) -> bool {
        if *current_fitness < 0.0 {
            return true;
        }

        let delta_solutions = (proposed_fitness - current_fitness) as f64;

        if delta_solutions < 0.0 {
            true
        }
        else {
            let mut rng = thread_rng();
            let niu = rng.gen();
            let p = (((-1 as f64) * delta_solutions) / self.current_temp).exp();

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