use chrono::{DateTime, Utc};
use nalgebra::{Matrix6, Vector3};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::ros_msg::Ros1;



#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Twist {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
    pub covariance: Option<Matrix6<f64>>,
}

impl Ros1 for Twist {
    fn empty() -> Twist{
        Twist { 
            linear: Vector3::zeros(), 
            angular: Vector3::zeros(), 
            covariance: None 
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<Twist, RosError>{
        let l_x = yaml["linear"]["x"].as_f64().unwrap();
        let l_y = yaml["linear"]["y"].as_f64().unwrap();
        let l_z = yaml["linear"]["z"].as_f64().unwrap();

        let a_x = yaml["angular"]["x"].as_f64().unwrap();
        let a_y = yaml["angular"]["y"].as_f64().unwrap();
        let a_z = yaml["angular"]["z"].as_f64().unwrap();

        let linear = Vector3::from([l_x, l_y, l_z]);
        let angular = Vector3::from([a_x, a_y, a_z]);

        let mut twist =  Twist{
            linear,
            angular,
            covariance: None
        };

        let mut covariance: Vec<f64> = vec![];

        let _: Vec<_> = yaml["covariance"]
            .as_vec()
            .unwrap_or(
                return Ok(twist)
            )
            .iter()
            .map(|y|{
                covariance.push(y.as_f64().unwrap());
            })
            .collect();

        twist.covariance = Some(Matrix6::from_vec(covariance));


        return Ok(twist)

    }
}