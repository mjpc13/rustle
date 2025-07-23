use serde::{Serialize, Deserialize};
use super::test_definition::Sensor;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DropParams {
    pub drop_list: Vec<Drop>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Drop {
    pub sensor: Sensor,
    pub drop_rate: [u32; 2],
    pub topic: String,
    #[serde(default)]
    pub active_periods: Vec<ActivePeriod>,
}



impl DropParams {

    pub fn update_file(&self, yaml_content: &str) -> String {
        let mut modified_content = yaml_content.to_string();
        
        for drop in &self.drop_list {
            // Replace original topic with topic_drop
            modified_content = modified_content.replace(
                &format!("{}", drop.topic),
                &format!("{}_drop", drop.topic)
            );

        }

        let drops_yaml = serde_yaml::to_string(self).unwrap();

        modified_content.push_str(&format!("\n{}\n", drops_yaml));

        
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repeat: Option<RepeatConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepeatConfig {
    #[serde(rename = "interval_sec")]
    pub interval: u32,
    #[serde(rename = "count")]
    pub repetitions: u32,
}