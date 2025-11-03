use nalgebra::{Matrix6, Point3, Quaternion};
use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::{ros_msg::Ros1, Header};


#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Pose {
    pub position: Point3<f64>,
    pub orientation: Quaternion<f64>,
    pub covariance: Option<Matrix6<f64>>,
}

#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct PoseStamped
{
    pub header: Header,
    pub pose: Pose
}

impl Ros1 for Pose {
    fn empty() -> Pose{
        Pose { 
            position: Point3::origin(), 
            orientation: Quaternion::identity(), 
            covariance: None 
        }
    }
    fn from_yaml(yaml: Yaml) -> Result<Pose, RosError>{

        let p_x = yaml["position"]["x"].as_f64().expect(&format!("{:#?}", yaml["position"]));
        let p_y = yaml["position"]["y"].as_f64().unwrap();
        let p_z = yaml["position"]["z"].as_f64().unwrap();

        let o_x = yaml["orientation"]["x"].as_f64().unwrap();
        let o_y = yaml["orientation"]["y"].as_f64().unwrap();
        let o_z = yaml["orientation"]["z"].as_f64().unwrap();
        let o_w = yaml["orientation"]["w"].as_f64().unwrap();


        let position = Point3::from([p_x, p_y, p_z]);
        let orientation = Quaternion::from([o_x, o_y, o_z, o_w]);

        let pose =  Pose{
            position,
            orientation,
            covariance: None
        };

        let mut covariance: Vec<f64> = vec![];

        let _: Vec<_> = yaml["covariance"]
            .as_vec()
            .unwrap_or(
                return Ok(pose)
            )
            .iter()
            .map(|y|{
                covariance.push(y.as_f64().unwrap());
            })
            .collect();

        pose.covariance = Some(Matrix6::from_vec(covariance));

        return Ok(pose)

    }

    fn from_json(value: &serde_json::Value) -> Result<Pose, RosError> {
        // Parse position
        let p_x = value["position"]["x"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'position.x': {:?}", value)))?;
        let p_y = value["position"]["y"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'position.y': {:?}", value)))?;
        let p_z = value["position"]["z"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'position.z': {:?}", value)))?;

        // Parse orientation
        let o_x = value["orientation"]["x"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'orientation.x': {:?}", value)))?;
        let o_y = value["orientation"]["y"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'orientation.y': {:?}", value)))?;
        let o_z = value["orientation"]["z"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'orientation.z': {:?}", value)))?;
        let o_w = value["orientation"]["w"].as_f64()
            .ok_or_else(|| RosError::FormatError(format!("Missing 'orientation.w': {:?}", value)))?;

        let position = Point3::from([p_x, p_y, p_z]);
        let orientation = Quaternion::from([o_x, o_y, o_z, o_w]);

        let mut pose = Pose {
            position,
            orientation,
            covariance: None,
        };

        // Parse covariance if present
        if let Some(cov_vec) = value["covariance"].as_array() {
            let covariance: Result<Vec<f64>, RosError> = cov_vec.iter()
                .map(|v| v.as_f64().ok_or_else(|| 
                    RosError::FormatError(format!("Invalid covariance value: {:?}", v))
                ))
                .collect();
            
            pose.covariance = Some(Matrix6::from_vec(covariance?));
        }

        Ok(pose)
    }


}

impl Ros1 for PoseStamped {
    fn empty() -> PoseStamped{
        PoseStamped{
            header: Header::empty(),
            pose: Pose::empty()
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<PoseStamped, RosError>{
        
        let header = Header::from_yaml(yaml["header"].clone())?;
        let pose = Pose::from_yaml(yaml["pose"].clone())?;

        return Ok(PoseStamped{
            header,
            pose
        });
    }

    fn from_json(value: &serde_json::Value) -> Result<Self, RosError> {
        let header = Header::from_json(&value["header"].clone())?;
        let pose = Pose::from_json(&value["pose"].clone())?;

        return Ok(PoseStamped{
            header,
            pose
        });
    }
}
