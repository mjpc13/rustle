use std::collections::HashMap;
use serde_json::Value;
use surrealdb::sql::Thing;
use std::fs::{File, self};
use std::io::{BufReader, Write, BufRead};

use crate::models::parameter_space::ParameterSpaceThing;
use crate::models::slam_config::SLAMConfig;
use crate::services::error::{BoolParsingError, NumberParsingError, ParameterSpaceError, StringParsingError};

use std::mem::discriminant;

#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq)]
pub struct TuningConfig {
    pub id: Option<Thing>,
    pub algo_id: Option<Thing>,
    pub dataset_id: Option<Thing>,
    pub parameters: HashMap<String, Value>,
}

impl TuningConfig {
    pub fn new() -> Self {
        Self { id: None, algo_id: None, dataset_id: None, parameters: HashMap::new()}
    }

    /// This function assumes `algo_cfg` belongs to the correct algorithm.
    /// This means it is on the caller to pass an actual SLAM configuration HashMap as an argument
    pub fn validate_parameter_space(&self, algo_cfg: &HashMap<String, Value>) -> Result<(), ParameterSpaceError> {
        
        for (key, value) in &self.parameters {

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
}

pub fn find_key<'a>(value: &'a Value, target: &str) -> Option<&'a Value> {
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

pub fn find_key_in_hash_map<'a>(map: &'a HashMap<String, Value>, target: &str) -> Option<&'a Value> {
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