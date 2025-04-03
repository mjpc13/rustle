use bollard::image;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Algorithm {
    pub id: Option<Thing>,           // Use SurrealDB-style ID (e.g., "algorithm:orb_slam3")
    pub name: String,
    pub image_name: String,
    pub version: String,
    pub parameters: String,    // Change to YAML
    pub odom_topics: Vec<String>
}

impl Algorithm {
    /// Constructor with automatic ID generation
    pub fn new(name: String, version: String, image_name: String, parameters: String, odom_topics: Vec<String>) -> Self {


        //Logic to parse file to a YAML object!
        //YAML_obj

        Self {
            id: None,
            name,
            image_name,
            version,
            parameters,
            odom_topics
        }
    }
}