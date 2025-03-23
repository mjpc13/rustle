use bollard::image;
use serde::{Serialize, Deserialize};
use serde_yaml::Value;
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Algorithm {
    pub id: Option<Thing>,           // Use SurrealDB-style ID (e.g., "algorithm:orb_slam3")
    pub name: String,
    pub image_name: String,
    pub version: String,
    pub parameters: String,    // Probably will be better for Francisco to be a Value...
}

impl Algorithm {
    /// Constructor with automatic ID generation
    pub fn new(name: String, version: String, image_name: String, parameters: String) -> Self {
        Self {
            id: None,
            name,
            image_name,
            version,
            parameters,
        }
    }
}