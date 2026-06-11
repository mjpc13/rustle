use std::collections::HashMap;
use serde_json::{Value, Map};
use surrealdb::sql::Thing;
use std::fs::{File, self};
use std::io::{BufReader, Write, BufRead};

use crate::models::slam_config::SLAMConfig;
use crate::services::error::{BoolParsingError, NumberParsingError, ParameterSpaceError, StringParsingError};
use crate::models::tuning::grid_search::GridSearchConfig;

use crate::models::metrics::Metric;

use crate::models::tuning::TuneType;
use rand::{Rng, thread_rng};

use csv::Writer;

#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq)]
pub struct TuningConfig {
    pub id: Option<Thing>,

    pub name: Option<String>,
    
    pub tuning_algo: Option<String>,
    pub algo_id: Option<Thing>,
    pub dataset_id: Option<Thing>,
    
    pub parameters_to_tune: HashMap<String, Value>,
    
    pub tuning_type: Option<TuneType>,
    
    pub results: Vec<(Vec<Metric>, usize)>,

    pub dataset_settings: (Option<f32>, Option<f32>), // (start, duration)
    
    pub metrics_weights: Option<(f32, f32)>, // (ape, rpe)
    
    pub early_stopping_params: Option<(u64, f32)>,

    /// time limit for tuning, in hours
    pub time_limit: Option<f64>,
}

impl TuningConfig {
    pub fn new() -> Self {
        Self { id: None, 
               name: None,
               tuning_algo: None, 
               algo_id: None, 
               dataset_id: None, 
               parameters_to_tune: HashMap::new(),

               tuning_type: None,
               
               results: Vec::new(),
               
               dataset_settings: (None, None),

               metrics_weights: None, 
               early_stopping_params: None, 
               time_limit: None, }
    }

    /// This function assumes `algo_cfg` belongs to the correct algorithm.
    /// This means it is on the caller to pass an actual SLAM configuration HashMap as an argument
    pub fn validate_parameter_space(&self, algo_cfg: &mut HashMap<String, Value>) -> Result<(), ParameterSpaceError> {
        
        for (key, value) in &self.parameters_to_tune {

            if let Some(value_gt) = find_key_in_hash_map(algo_cfg, key) {
                match value_gt {
                    Value::String(s) => self.validate_string(value, key.clone()),
                    Value::Bool(b) => self.validate_bool(value, key.clone()),
                    // Value::Number could be: u64, i64 or f64
                    Value::Number(number) => {
                        if number.is_u64() {
                            self.validate_u64(value, key.clone())
                        }
                        else if number.is_i64() {
                            self.validate_i64(value, key.clone())
                        }
                        else {
                            self.validate_f64(value, key.clone())
                        }
                    },
                    Value::Array(arr) => {
                        let arr_vec = value_gt.as_array().unwrap();
                        self.validate_array(value.clone(), arr_vec, key.clone())
                    }
                    _ => Ok(()),
                }?
            }
            else {
                return Err(ParameterSpaceError::InvalidKey(key.clone()));
            }

        }

        Ok(())
    }

    fn validate_string(&self, value: &Value, key: String) -> Result<(), ParameterSpaceError> {
        match value {
            Value::String(_) => Ok(()),
            Value::Array(elements) => {
                let temp_vec = value.as_array().unwrap();

                if temp_vec.len() == 0 {
                    return Err(ParameterSpaceError::StringParsing(StringParsingError::EmptyStringArray(key)));
                }

                for i in 0..temp_vec.len() {
                    if !temp_vec[i].is_string() {
                        return Err(ParameterSpaceError::StringParsing(StringParsingError::WrongTypeInArray(key)));
                    }

                    for j in i+1..temp_vec.len() {
                        if temp_vec[i] == temp_vec[j] {
                            return Err(ParameterSpaceError::StringParsing(StringParsingError::RepeatedValues(key)));
                        }
                    }
                }
                return Ok(());
            },
            _ => return Err(ParameterSpaceError::StringParsing(StringParsingError::NotAString(key))),
        }
    }

    fn validate_bool(&self, value: &Value, key: String) -> Result<(), ParameterSpaceError> {
        match value {
            Value::Bool(_) => Ok(()),
            Value::Array(elements) => {
                let temp_vec = value.as_array().unwrap();

                if temp_vec.len() == 0 {
                    return Err(ParameterSpaceError::BoolParsing(BoolParsingError::EmptyBoolArray(key)));
                }

                if temp_vec.len() > 2 {
                    return Err(ParameterSpaceError::BoolParsing(BoolParsingError::TooManyValuesInArray(key)));
                }
                else if temp_vec.len() == 2 {
                    if !temp_vec[0].is_boolean() || !temp_vec[1].is_boolean() {
                        return Err(ParameterSpaceError::BoolParsing(BoolParsingError::WrongTypeInArray(key)));
                    }
                    else if temp_vec[0] == temp_vec[1] {
                        return Err(ParameterSpaceError::BoolParsing(BoolParsingError::RepeatedValues(key)));
                    }
                    return Ok(());
                }

                // only 1 value...
                if !temp_vec[0].is_boolean() {
                    return Err(ParameterSpaceError::BoolParsing(BoolParsingError::NotABoolean(key)));
                }
                Ok(())
            },
            _ => return Err(ParameterSpaceError::BoolParsing(BoolParsingError::NotABoolean(key))),
        }
    }

    fn validate_u64(&self, value: &Value, key: String) -> Result<(), ParameterSpaceError> {
        if value.is_u64() {
            Ok(())
        }
        else if value.is_array() {
            // temp_vec should be a vector of 3 elements:
            // first value
            // step
            // total number of values -> u64
            let temp_vec = value.as_array().unwrap();

            if temp_vec.len() != 3 {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongArrayFormat(key)));
            }

            if !temp_vec[0].is_u64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongFirstValue(String::from("u64"), key)));
            }

            if !temp_vec[1].is_u64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongStepType(String::from("u64"), key)));
            }

            if !temp_vec[2].is_u64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongNumElementsType(String::from("u64"), key)));
            }

            Ok(())
        }
        else {
            return Err(ParameterSpaceError::NumberParsing(NumberParsingError::NotANumber(String::from("u64"), key)));
        }
    }

    // not tested
    fn validate_i64(&self, value: &Value, key: String) -> Result<(), ParameterSpaceError> {
        if value.is_i64() {
            Ok(())
        }
        else if value.is_array() {
            // temp_vec should be a vector of 3 elements:
            // first value
            // step
            // total number of values -> u64
            let temp_vec = value.as_array().unwrap();

            if temp_vec.len() != 3 {
                return Err(ParameterSpaceError::NumberArrayWrongFormat());
            }
            else {
                if !temp_vec[0].is_i64() || !temp_vec[1].is_i64() || !temp_vec[2].is_i64() {
                    return Err(ParameterSpaceError::WrongTypeInArray(key, String::from("i64")))
                }
                if temp_vec[2].as_i64().unwrap() == 0 {
                    return Err(ParameterSpaceError::NoValuesInVector());
                }
            }
            Ok(())
        }
        else {
            return Err(ParameterSpaceError::IncompatibleTypes(String::from("i64")));
        }
    }

    fn validate_f64(&self, value: &Value, key: String) -> Result<(), ParameterSpaceError> {
        if value.is_f64() {
            Ok(())
        }
        else if value.is_array() {
            // temp_vec should be a vector of 3 elements:
            // first value
            // step
            // total number of values -> u64
            let temp_vec = value.as_array().unwrap();

            if temp_vec.len() != 3 {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongArrayFormat(key)));
            }

            if !temp_vec[0].is_f64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongFirstValue(String::from("f64"), key)));
            }

            if !temp_vec[1].is_f64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongStepType(String::from("f64"), key)));
            }

            if !temp_vec[2].is_u64() {
                return Err(ParameterSpaceError::NumberParsing(NumberParsingError::WrongNumElementsType(String::from("f64"), key)));
            }

            Ok(())
        }
        else {
            return Err(ParameterSpaceError::NumberParsing(NumberParsingError::NotANumber(String::from("f64"), key)));
        }
    }

    /// For the moment, this function only allows 1 array to be passed(of numeric types, f64, i64, or u64)
    /// 
    /// Could add more flexibility in the future, but this is enough for now
    fn validate_array(&self, value: Value, value_gt: &Vec<Value>, key: String) -> Result<(), ParameterSpaceError> {

        if !value.is_array() {
            return Err(ParameterSpaceError::NotAnArray(key));
        }

        let value_arr = value.as_array().unwrap();

        if value_arr.len() != value_gt.len() {
            return Err(ParameterSpaceError::WrongArraySize(key, value_gt.len(), value_arr.len()));
        }

        for v in value_arr {
            if value_gt[0].is_i64() {
                if !v.is_i64() {
                    return Err(ParameterSpaceError::WrongTypeInArray(key, String::from("i64")));
                }
            }
            else if value_gt[0].is_u64() {
                if !v.is_u64() {
                    return Err(ParameterSpaceError::WrongTypeInArray(key, String::from("u64")));
                }
            }
            else if value_gt[0].is_f64() {
                if !v.is_f64() {
                    return Err(ParameterSpaceError::WrongTypeInArray(key, String::from("f64")));
                }
            }
        }

        Ok(())
    }

    /*
    /// only for random search
    pub fn generate_random_point(&mut self) -> Option<Vec<(String, u64)>> {
        let mut rs_config = match &mut self.tuning_type {
            Some(TuneType::RandomSearch(rs_config_thing)) => rs_config_thing,
            _ => {&mut GridSearchConfig::new()}
        };

        let params_array_sizes = rs_config.tunable_params_array_sizes.clone();

        let total_size = rs_config.get_param_array_size_by_index(None).unwrap();
        let mut random_point: Vec<(String, u64)> = Vec::new();
        let mut rng = thread_rng();

        if rs_config.visited_points.len() == rs_config.get_param_array_size_by_index(None).unwrap() as usize {
            return None;
        }

        loop {
                        
            for (key, param_size) in params_array_sizes.clone() {
                random_point.push((key.clone(), rng.gen_range(0..param_size.clone())));
            }

                        //  && !(self.get_grid_point_index(&random_point).unwrap() < rs_config.get_param_array_size_by_index(None).unwrap())
                        /*
                        if !rs_config.visited_points.contains(&random_point) {
                            rs_config.visited_points.push(random_point.clone());
                            return Some(random_point);
                        }
                        */
            if self.get_grid_point_index(&random_point, &params_array_sizes).unwrap() < total_size {
                break;
            }
            else if !rs_config.visited_points.contains(&random_point) {
                break;
            }
            else {
                random_point.clear();
            }
        }

        rs_config.add_visited_point(&random_point);

        /*
        if let Some(rs_config_type) = &mut self.tuning_type {
            match rs_config_type {
                TuneType::RandomSearch(rs_config) => {
                    let total_size = rs_config.get_param_array_size_by_index(None).unwrap();
                    let mut random_point: Vec<(String, u64)> = Vec::new();
                    let mut rng = thread_rng();

                    if rs_config.visited_points.len() == rs_config.get_param_array_size_by_index(None).unwrap() as usize {
                        return None;
                    }

                    loop {
                        
                        for (key, param_size) in rs_config.tunable_params_array_sizes.clone() {
                            random_point.push((key.clone(), rng.gen_range(0..param_size.clone())));
                        }

                        //  && !(self.get_grid_point_index(&random_point).unwrap() < rs_config.get_param_array_size_by_index(None).unwrap())
                        /*
                        if !rs_config.visited_points.contains(&random_point) {
                            rs_config.visited_points.push(random_point.clone());
                            return Some(random_point);
                        }
                        */
                        if !(self.get_grid_point_index(&random_point).unwrap() < self.get_param_array_size_by_index(None).unwrap()) {

                        }
                        else {
                            random_point.clear();
                        }
                    }
                }
                _ => None
            }
        }
        else {
            None
        }
        */
        None
    }
    */

    pub fn get_grid_point_index(&self, random_point: &Vec<(String, u64)>, params_array_sizes: &Vec<(String, u64)>) -> Option<u64> {
        /*
        if let Some(TuneType::RandomSearch(rs_config)) = &self.tuning_type {
            let mut total_index: u64 = 0;
            for i in 0..random_point.len() {
                //total_index += random_point[i].1 * (rs_config.get_param_array_size_by_index(Some(i)).unwrap()).pow(i as u32);
                total_index += random_point[i].1 * params_array_sizes.1.pow(i as u32);
                //total_index += random_point[i].1 * (rs_config.tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
            }
            Some(total_index)
        }
        else if let Some(TuneType::GridSearch(gs_config)) = &self.tuning_type {
            let mut total_index: u64 = 0;
            for i in 0..random_point.len() {
                //total_index += random_point[i].1 * (gs_config.tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
                total_index += random_point[i].1 * (gs_config.get_param_array_size_by_index(Some(i)).unwrap()).pow(i as u32);
            }
            Some(total_index)
        }
        else { None }
        */
        let mut total_index: u64 = 0;
        for i in 0..random_point.len() {
            //total_index += random_point[i].1 * (rs_config.get_param_array_size_by_index(Some(i)).unwrap()).pow(i as u32);
            total_index += random_point[i].1 * params_array_sizes[i].1.pow(i as u32);
            //total_index += random_point[i].1 * (rs_config.tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
        }
        Some(total_index)
    }

    pub fn get_param_array_size_by_index(&self, index: Option<usize>) -> Option<u64> {
        if let Some(TuneType::RandomSearch(rs_config)) = &self.tuning_type {
            if let Some(index_value) = index {
                if index_value >= rs_config.tunable_params_array_sizes.len() {
                    return None;
                }
                else {
                    return Some(rs_config.tunable_params_array_sizes[index_value].1.clone());
                }
            }
            else {
                return Some(rs_config.tunable_params_array_sizes.iter().map(|(_, v)| v.clone()).product());  
            }
        }
        else { return None; }
    }

    pub fn is_random_point_index_valid(&self, random_point_index: &u64) -> bool {
        if let Some(TuneType::RandomSearch(rs_config)) = &self.tuning_type {
            if *random_point_index >= rs_config.get_param_array_size_by_index(None).unwrap() {
                return false;
            }
        }
        true
    }

    /// only for grid search
    pub fn generate_next_grid_point(&self, previous_point: Vec<(String, u64)>) -> Option<Vec<(String, u64)>> {
        if let Some(gs_config_type) = &self.tuning_type {
            match gs_config_type {
                TuneType::GridSearch(gs_config) => {
                    let params_array_sizes = &gs_config.tunable_params_array_sizes;
                    let mut next_point = previous_point.clone();
                    let first_element_size = params_array_sizes[0].1.clone();
                    if next_point[0].1.clone() == (first_element_size - 1) {
                        let first_key = next_point[0].0.clone();
                        change_parameter_array_index(&mut next_point, first_key, 0);
                        for i in 1..next_point.len() {
                            let current_key = next_point[i].0.clone();
                            let current_param_index: u64 = get_parameter_array_current_index(next_point.clone(), current_key.clone()).unwrap();
                            if current_param_index == gs_config.get_param_array_size_by_key(Some(&current_key)).unwrap() - 1 {
                                change_parameter_array_index(&mut next_point, current_key, 0);
                            }
                            else {
                                change_parameter_array_index(&mut next_point, current_key.clone(), current_param_index + 1);
                                break;
                            }
                        }
                        return Some(next_point);
                    }
                    else {
                        let first_key = next_point[0].0.clone();
                        let first_value = next_point[0].1.clone();
                        change_parameter_array_index(&mut next_point, first_key, first_value + 1);
                        return Some(next_point);
                    }
                    Some(next_point)
                }
                _ => None
            }
        }
        else { None }
    }

    pub fn generate_initial_grid_point(&self) -> Option<Vec<(String, u64)>> {
        if let Some(gs_config_type) = &self.tuning_type {
            match gs_config_type {
                TuneType::GridSearch(gs_config) => {
                    let mut first_point: Vec<(String, u64)> = Vec::new();

                    for (key, value) in &gs_config.tunable_params_array_sizes {
                        first_point.push((key.clone(), 0));
                    }

                    Some(first_point)
                }
                _ => { None }
            }
        }
        else { None }
    }

    /// only for grid/random search
    pub fn get_current_params(&self, grid_point: &Vec<(String, u64)>, initial_params: &HashMap<String, Value>) -> Option<HashMap<String, Value>> {
        //println!("{:?}", grid_point.clone());
        //println!("{:?}", &self.tuning_type);
        if let Some(config_type) = &self.tuning_type {
            //println!("Its something");
            match config_type {
                TuneType::RandomSearch(rs_config) => {
                    let tunable_params = &rs_config.tunable_params;
                    let mut current_params = &mut initial_params.clone();

                    for (key, value) in tunable_params {
                        //match current_params.get(key).unwrap() {
                        match find_key_in_hash_map(current_params, &key) {
                            Some(Value::Number(n)) => {
                                if n.is_u64() {
                                    let unsigned_int_tuple = u64_array_tuple(value);
                                    //let next_value: u64 = unsigned_int_tuple.0 + grid_point.get(key).unwrap().clone() * unsigned_int_tuple.1;
                                    let next_value: u64 = unsigned_int_tuple.0 + get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() * unsigned_int_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                                else if n.is_i64() {
                                    let int_tuple = i64_array_tuple(value);
                                    //let next_value: i64 = (int_tuple.0) + (grid_point.get(key).unwrap().clone() as i64 * int_tuple.1);
                                    let next_value: i64 = int_tuple.0 + (get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as i64) * int_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                                else if n.is_f64() {
                                    let float_tuple = f64_array_tuple(value);
                                    //let next_value: f64 = float_tuple.0 + (grid_point.get(key).unwrap().clone() as f64) * float_tuple.1;
                                    let next_value: f64 = float_tuple.0 + (get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as f64) * float_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                            }
                            _ => return None
                        }
                    }
                    return Some(current_params.clone());
                }
                TuneType::GridSearch(gs_config) => {
                    //println!("Its grid search");
                    let tunable_params: HashMap<String, Value> = gs_config.tunable_params.clone();
                    let mut current_params = &mut initial_params.clone();

                    for (key, value) in tunable_params {
                        //println!("{:?}", value.clone());
                        //match current_params.get(&key) {
                        match find_key_in_hash_map(current_params, &key) {
                            Some(Value::Number(n)) => {
                                if n.is_u64() {
                                    //println!("{} is u64", key.clone());
                                    let unsigned_int_tuple = u64_array_tuple(&value);
                                    //let next_value: u64 = unsigned_int_tuple.0 + point.get(&key).unwrap().clone() * unsigned_int_tuple.1;
                                    //println!("{:?}: {}", grid_point.clone(), key.clone());
                                    let next_value: u64 = unsigned_int_tuple.0 + get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() * unsigned_int_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                                else if n.is_i64() {
                                    //println!("{} is i64", key.clone());
                                    let int_tuple = i64_array_tuple(&value);
                                    //let next_value: i64 = (int_tuple.0) + (point.get(&key).unwrap().clone() as i64 * int_tuple.1);
                                    let next_value: i64 = int_tuple.0 + (get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as i64) * int_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                                else if n.is_f64() {
                                    //println!("{} is f64", key.clone());
                                    let float_tuple = f64_array_tuple(&value);
                                    //let next_value: f64 = float_tuple.0 + (point.get(&key).unwrap().clone() as f64) * float_tuple.1;
                                    let next_value: f64 = float_tuple.0 + (get_parameter_array_current_index(grid_point.clone(), key.clone()).unwrap() as f64) * float_tuple.1;
                                    current_params.insert(key.clone(), Value::from(next_value));
                                }
                            }
                            _ => {
                                //println!("Not a number");
                                return None
                            }
                        }
                    }
                    //println!("After getting the params");
                    return Some(current_params.clone());
                }
                _ => {
                    //println!("Something else");
                    return None
                }
            }
        }
        else {
            None
        }
    }

    pub fn save_tuning_progress(&self, writer: &mut Writer<File>, current_iter_data: &(usize, f32, f32), iter_data: &Vec<String>) -> Result<(), Box<dyn std::error::Error>>{
        /*
        writer.write_record(&[
            current_iter_data.0.to_string(),
            current_iter_data.1.to_string(),
            current_iter_data.2.to_string(),
        ])?;
        */

        writer.write_record(iter_data)?;

        Ok(())
    }
}

pub fn find_key<'a>(value: &'a mut Value, target: &str) -> Option<&'a mut  Value> {
    match value {
        Value::Object(map) => {
            for (k, v) in map {
                match v {
                    Value::Object(_) => {
                        //println!("Found nested object: {}", k);
                        if let Some(found) = find_key(v, target) {
                            return Some(found);
                        }
                    }
                    _ => {
                        if k == target {
                            return Some(v);
                        }
                    }
                }
            }
            None
        },

        _ => None,
    }
}

pub fn format_string(arr: &Vec<Value>) -> String {
    let mut final_string = String::from("[");
    for i in 0..arr.len() {
        final_string.push_str(&arr[i].to_string());
        if i < arr.len() - 1 {
            final_string.push_str(", ");
        }
    }
    final_string.push_str("]");
    final_string
}

pub fn find_key_in_hash_map<'a>(map: &'a mut HashMap<String, Value>, target: &str) -> Option<&'a mut Value> {
    for (key, value) in map {
        if key == target {
            return Some(value);
        }

        if value.is_object() {
            if let Some(found_value) = find_key(value, target) {
                return Some(found_value);
            }
        }
    }
    None
}

fn format_hashmap_object(object: &mut Map<String, Value>) {
    for (key, value) in object.iter_mut() {
        if let Some(inner_object_map) = value.as_object_mut() {
            format_hashmap_object(inner_object_map);
        }
        else if let Some(array) = value.as_array() {
            *value = Value::String(format_string(array));
        }
    }
}

/// for propper yaml output, takes array values and transforms them into Strings(Value::String), starting and ending with [ ]
pub fn format_hashmap(params: &mut HashMap<String, Value>) {
    for (key, value) in params.iter_mut() {
        if let Some(inner_object_map) = value.as_object_mut() {
            format_hashmap_object(inner_object_map);
        }
        else if let Some(array) = value.as_array() {
            *value = Value::String(format_string(array));
        }
    }
}

fn u64_array_tuple(value: &Value) -> (u64, u64, u64) {
    let binding = value.clone();
    let array = binding.as_array().unwrap();
    let start: u64 = array[0].as_u64().unwrap();
    let step: u64 = array[1].as_u64().unwrap();
    let num_elements = array[2].as_u64().unwrap();

    (start, step, num_elements)
}

fn i64_array_tuple(value: &Value) -> (i64, i64, u64) {
    let binding = value.clone();
    let array = binding.as_array().unwrap();
    let start: i64 = array[0].as_i64().unwrap();
    let step: i64 = array[1].as_i64().unwrap();
    let num_elements = array[2].as_u64().unwrap();

    (start, step, num_elements)
}

fn f64_array_tuple(value: &Value) -> (f64, f64, u64) {
    let binding = value.clone();
    let array = binding.as_array().unwrap();
    let start: f64 = array[0].as_f64().unwrap();
    let step: f64 = array[1].as_f64().unwrap();
    let num_elements = array[2].as_u64().unwrap();

    (start, step, num_elements)
}

fn get_parameter_array_current_index(grid_point: Vec<(String, u64)>, target_key: String) -> Option<u64> {
    if let Some((key, index)) = grid_point.iter().find(|(key, _)| *key == target_key) {
        return Some(index.clone());
    }
    None
}

fn change_parameter_array_index(grid_point: &mut Vec<(String, u64)>, target_key: String, new_value: u64) {
    if let Some((key, index)) = grid_point.iter_mut().find(|(key, _)| *key == target_key) {
        *index = new_value;
    }
}
