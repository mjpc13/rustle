use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::{ros_msg::Ros1, Header, PoseStamped};


#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Path
{
    pub header: Header,
    pub poses: Vec<PoseStamped>
}

impl Ros1 for Path {
    fn empty() -> Path{
        Path { 
            header: Header::empty(), 
            poses: Vec::new() 
        }
    }

    fn from_yaml(yaml: Yaml) -> Result<Path, RosError>{
        
        let header = Header::from_yaml(yaml["header"].clone())?;

        let poses: Vec<PoseStamped> = yaml["poses"].clone()
            .into_iter()
            .map(|y|{
                PoseStamped::from_yaml(y).unwrap()
            })
            .collect();

        Ok(
            Path{
                header,
                poses
            }
        )   
    }

    fn from_json(value: &serde_json::Value) -> Result<Path, RosError> {
        // Parse header
        let header = Header::from_json(&value["header"])?;

        // Parse poses array
        let poses_array = value["poses"].as_array()
            .ok_or_else(|| RosError::FormatError(format!("Expected 'poses' array in Path: {:?}", value)))?;

        let mut poses = Vec::with_capacity(poses_array.len());
        for p in poses_array {
            poses.push(PoseStamped::from_json(p)?);
        }

        Ok(Path {
            header,
            poses,
        })
    }

}