use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use surrealdb::sql::Thing;
use std::{fmt, hash::{DefaultHasher, Hash, Hasher}};

use crate::models::{metric::StatisticalMetricsStamped, TestDefinition};

use super::{metric::Metric, Algorithm};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmRun {
    pub id: Option<Thing>,
    pub algo: Algorithm,
    pub bag_speed: f32,
    pub num_iterations: u8,
    pub metrics: Vec<Metric>, 
    pub cpu_load_list: Vec<StatisticalMetricsStamped>,
    pub mem_usage_list: Vec<StatisticalMetricsStamped>,
    pub ape_list: Vec<StatisticalMetricsStamped>,
    pub rpe_list: Vec<StatisticalMetricsStamped>,
    pub created_at: DateTime<Utc>,
    pub duration_secs: f64,
    pub test_type: String
}

impl AlgorithmRun {
    pub fn new(bag_speed: f32, num_iterations: u8, algo: Algorithm, test_type: String) -> Self {

        Self{
            id: None,
            bag_speed,
            created_at: Utc::now(),
            num_iterations,
            duration_secs: 0.0,
            metrics: Vec::new(),
            algo: algo,
            cpu_load_list: vec![],
            mem_usage_list: vec![],
            ape_list: vec![],
            rpe_list: vec![],
            test_type,
        }
    }

    pub fn get_distinct_rgba(&self, alpha: f32) -> String {
        let mut algo_hasher = DefaultHasher::new();

        if let Some(ref id) = self.algo.id {
            id.hash(&mut algo_hasher);
        } else {
            self.algo.name.hash(&mut algo_hasher);
        }
        // Main color is given by the algorithm type
        let algo_hash = algo_hasher.finish();
        let base_hue = (algo_hash % 360) as f64;

        // Hash run speed to derive a small offset in color
        let mut run_hasher = DefaultHasher::new();
        self.bag_speed.to_bits().hash(&mut run_hasher);
        let run_hash = run_hasher.finish();

        // Limit color offset to +/- 15 degrees THIS MIGHT BE IN THE CONFIG FILE!!!
        let offset = ((run_hash % 30) as i64) - 15;
        let final_hue = ((base_hue as i64 + offset + 360) % 360) as f64;

        let (r, g, b) = hsl_to_rgb(final_hue / 360.0, 0.7, 0.5);
        format!("rgba({}, {}, {}, {})", r, g, b, alpha)
    }

}

impl PartialEq for AlgorithmRun {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for AlgorithmRun {}

impl Hash for AlgorithmRun {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
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

impl fmt::Display for AlgorithmRun {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}_{}x", self.algo.name, self.bag_speed)
    }
}