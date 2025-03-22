use serde::{Serialize, Deserialize};
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Container {
    pub id: String,          // Format: "container:<ulid>"
    pub image_name: String,
    pub name: String,
    pub tag: String,
    pub url: String,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

impl Container {
    pub fn new(image_name: String, name: String, tag: String, url: String) -> Self {
        let now = Utc::now();
        Self {
            id: format!("container:{}", ulid::Ulid::new()),
            image_name,
            name,
            tag,
            url,
            created_at: now,
            updated_at: now,
        }
    }

    /// Validate container URL format
    pub fn validate_url(&self) -> Result<(), String> {
        if self.url.starts_with("http://") || self.url.starts_with("https://") {
            Ok(())
        } else {
            Err("Invalid container URL format".into())
        }
    }
}