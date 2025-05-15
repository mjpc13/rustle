use std::collections::HashMap;

use chrono::Utc;
use log::{info, warn};
use serde::{Serialize, Deserialize};

use super::test_definition::Sensor;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CutParams {
    pub cut_list: Vec<Cut>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cut {
    pub sensor: Sensor,
    pub topic: String,
    #[serde(default)]
    pub active_periods: Vec<ActivePeriod>,
}



impl CutParams {

    pub fn update_file(&self, yaml_content: &str) -> String {
        let mut modified_content = yaml_content.to_string();
        
        for cut in &self.cut_list {
            // Replace original topic with topic_cut
            modified_content = modified_content.replace(
                &format!("{}", cut.topic),
                &format!("{}_cut", cut.topic)
            );

        }

        let cuts_yaml = serde_yaml::to_string(self).unwrap();

        modified_content.push_str(&format!("\n{}\n", cuts_yaml));
        
        
        modified_content
    }

}



//To implement in the future? Maybe???
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivePeriod {
    #[serde(rename = "start_time")]
    pub start_sec: u32,
    #[serde(rename = "duration")]
    pub duration_sec: u32,
}