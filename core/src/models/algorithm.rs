use std::hash::{DefaultHasher, Hash, Hasher};

use bollard::image;
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize,Eq, Hash, PartialEq)]
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

    pub fn get_color(&self) -> String {
        // Hash the algorithm name or id string
        let key = self.id.as_ref()
            .map(|thing| thing.to_string())
            .unwrap_or_else(|| self.name.clone());

        let mut hasher = DefaultHasher::new();
        key.hash(&mut hasher);
        let hash = hasher.finish();

        let hue = (hash % 360) as u16;  // Hue in degrees
        let saturation = 70;            // In percent
        let lightness = 50;             // In percent

        format!("hsl({}, {}%, {}%)", hue, saturation, lightness)
    }

    // Optionally, a semi-transparent version for area fill
    pub fn get_rgba(&self, alpha: f32) -> String {
        let key = self.id.as_ref()
            .map(|thing| thing.to_string())
            .unwrap_or_else(|| self.name.clone());

        let mut hasher = DefaultHasher::new();
        key.hash(&mut hasher);
        let hash = hasher.finish();

        let hue = (hash % 360) as f64;
        let (r, g, b) = hsl_to_rgb(hue / 360.0, 0.7, 0.5); // Convert to 0–1 range
        format!("rgba({}, {}, {}, {})", r, g, b, alpha)
    }
}

fn hsl_to_rgb(h: f64, s: f64, l: f64) -> (u8, u8, u8) {
    let a = s * f64::min(l, 1.0 - l);
    let f = |n: f64| {
        let k = (n + h * 12.0) % 12.0;
        let color = l - a * f64::max(f64::min(f64::min(k - 3.0, 9.0 - k), 1.0), -1.0);
        (color * 255.0).round() as u8
    };
    (f(0.0), f(8.0), f(4.0))
}
