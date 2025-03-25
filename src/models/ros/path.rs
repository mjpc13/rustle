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
}