use nalgebra::{Matrix6, Vector3};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::RosVersion;
use super::ros_msg::RosData;



#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Twist {
    pub linear: Vector3<f64>,
    pub angular: Vector3<f64>,
    pub covariance: Option<Matrix6<f64>>,
}

impl RosData for Twist {
    fn empty() -> Twist{
        Twist { 
            linear: Vector3::zeros(), 
            angular: Vector3::zeros(), 
            covariance: None 
        }
    }

    fn from_yaml(yaml: Yaml, _ros_version: RosVersion) -> Result<Twist, RosError>{
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

        if let Some(cov) = yaml["covariance"].as_vec() {
            twist.covariance = Some(Matrix6::from_vec(
                cov.iter()
                    .map(|v| v.as_f64().unwrap())
                    .collect()
            ));
        }

        Ok(twist)
    }


    fn from_json(value: &serde_json::Value, _ros_version: RosVersion) -> Result<Twist, RosError> {
        // Parse linear components
        let l_x = value["linear"]["x"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'linear.x': {:?}", value)))?;
        let l_y = value["linear"]["y"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'linear.y': {:?}", value)))?;
        let l_z = value["linear"]["z"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'linear.z': {:?}", value)))?;

        // Parse angular components
        let a_x = value["angular"]["x"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'angular.x': {:?}", value)))?;
        let a_y = value["angular"]["y"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'angular.y': {:?}", value)))?;
        let a_z = value["angular"]["z"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'angular.z': {:?}", value)))?;

        let linear = Vector3::from([l_x, l_y, l_z]);
        let angular = Vector3::from([a_x, a_y, a_z]);

        let mut twist = Twist {
            linear,
            angular,
            covariance: None,
        };

        // Parse covariance if present
        if let Some(cov) = value["covariance"].as_array() {
            twist.covariance = Some(Matrix6::from_vec(
                cov.iter()
                    .map(|v| v.as_f64().ok_or_else(||
                        RosError::FormatError(format!("Invalid covariance value: {:?}", v))
                    ))
                    .collect::<Result<Vec<f64>, RosError>>()?
            ));
        }

        Ok(twist)
    }
}
