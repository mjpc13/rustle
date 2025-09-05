use std::collections::HashMap;
use serde_json::Value;
use chrono::{DateTime, Utc};
use std::fs::{File, self};
use std::io::{BufReader, Write, BufRead};
use surrealdb::sql::Thing;

#[derive(Debug, serde::Deserialize, serde::Serialize, Clone, PartialEq)]
pub struct SLAMConfig {
    #[serde(default)]
    pub params: HashMap<String, Value>,
    pub id: Option<Thing>,
    #[serde(default)]
    created_at: DateTime<Utc>,
}

impl SLAMConfig {
    pub fn new(params_file: &str) -> Result<Self, Box<dyn std::error::Error>> {
        match Self::parse_yaml(params_file) {
            Ok(result) => Ok(Self {
                params: result.params,
                created_at: Utc::now(),
                id: None,
            }),
            Err(e) =>  {
                println!("An error ocurred while trying to read the SLAM parameter configuration file.");
                Err(e)
            }
        }
    }

    

    fn parse_yaml(file_path: &str) -> Result<SLAMConfig, Box<dyn std::error::Error>> {
        let file = File::open(file_path)?;
        let reader = BufReader::new(file);

        let mut config = SLAMConfig {params: HashMap::new(), id: None, created_at: Utc::now() };
        config.params = serde_yaml::from_reader(reader)?;

        Ok(config)
    }

    fn remove_quotes_from_target_keys(&self, file_path: &str, target_keys: Vec<String>) -> std::io::Result<()> {
        let file = File::open(file_path)?;
        let mut reader = BufReader::new(file);

        let mut target_keys_temp = Vec::new();
        for thing in target_keys {
            target_keys_temp.push(thing.clone());
        }

        // Store modified lines
        let mut lines: Vec<String> = Vec::new();

        for line in reader.lines() {
            let line = line?;
            if let Some((key, value)) = line.split_once(": ") {
                let key_temp = key.clone();
                //if target_keys_temp.contains(key) {
                if target_keys_temp.iter().any(|s| s == key) {
                    // Remove leading/trailing single quotes from value
                    let cleaned_value = value.trim_matches('\'');
                    lines.push(format!("{}: {}", key, cleaned_value));
                    continue;
                }
            }
            lines.push(line);
        }

        // Write modified content back to the file
        fs::write(file_path, lines.join("\n"))?;

        Ok(())
    }

    fn inspect_field_type(&self, field: &Value) -> &str {
        match field {
            Value::Number(_) => "Number",
            Value::String(_) => "String",
            Value::Bool(_) => "Bool",
            Value::Array(_) => "Array",
            Value::Object(_) => "Object",
            Value::Null => "Null",
        }
    }

    fn extract_numbers(&self, array: &Value) -> Option<Vec<f64>> {
        if let Value::Array(vec) = array {
            let mut result = Vec::new();
            for item in vec {
                if let Value::Number(n) = item {
                    result.push(n.as_f64().unwrap());
                } else {
                    return None;
                }
            }
            Some(result)
        } else {
            None
        }
    }

    pub fn save_to_file(&self, file_name: &str) {
        let mut file = File::create(file_name).unwrap();
        let yaml_string = serde_yaml::to_string(&self.params);
        file.write_all(yaml_string.unwrap().as_bytes());

        let mut updates = Vec::new();
        let mut stuff: HashMap<String, Value> = HashMap::new();
        stuff = self.params.clone();
        //let mut target_keys: Vec<&str> = Vec::new();
        let mut target_keys: Vec<String> = Vec::new();

        for (key, value) in &stuff {
            if self.inspect_field_type(&value) == "Array" {
                let mut vec = self.extract_numbers(&value).unwrap();
                let s = format!("{:?}", vec);
                updates.push((key.clone(), Value::String(s)));
                //target_keys.push(key.clone().as_str());
                target_keys.push(key.clone());
            }
        }

        for (key, new_value) in updates.clone() {
            stuff.insert(key, new_value);
        }

        let yaml_string = serde_yaml::to_string(&stuff);
        let mut file = File::create(file_name).unwrap();
        file.write_all(yaml_string.unwrap().as_bytes());

        self.remove_quotes_from_target_keys(file_name, target_keys);
    }
}

