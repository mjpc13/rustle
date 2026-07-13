use serde::{Deserialize, Serialize};
use yaml_rust2::Yaml;

use crate::services::RosError;

use super::RosVersion;
use super::{ros_msg::RosData, Header, PoseStamped};


#[derive(Debug, Serialize, Deserialize, Default, Clone)]
pub struct Path
{
    pub header: Header,
    pub poses: Vec<PoseStamped>
}

impl RosData for Path {
    fn empty() -> Path{
        Path { 
            header: Header::empty(), 
            poses: Vec::new() 
        }
    }

    fn from_yaml(yaml: Yaml, ros_version: RosVersion) -> Result<Path, RosError>{
        
        let header = Header::from_yaml(yaml["header"].clone(), ros_version)?;

        let poses: Vec<PoseStamped> = yaml["poses"].clone()
            .into_iter()
            .map(|y|{
                PoseStamped::from_yaml(y, ros_version).unwrap()
            })
            .collect();

        Ok(
            Path{
                header,
                poses
            }
        )   
    }

    fn from_json(value: &serde_json::Value, ros_version: RosVersion) -> Result<Path, RosError> {
        // Parse header
        let header = Header::from_json(&value["header"], ros_version)?;

        // Parse poses array
        let poses_array = value["poses"].as_array()
            .ok_or_else(|| RosError::FormatError(format!("Expected 'poses' array in Path: {:?}", value)))?;

        let mut poses = Vec::with_capacity(poses_array.len());
        for p in poses_array {
            poses.push(PoseStamped::from_json(p, ros_version)?);
        }

        Ok(Path {
            header,
            poses,
        })
    }

}
