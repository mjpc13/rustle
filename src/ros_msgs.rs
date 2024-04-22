use chrono::{DateTime, Utc};
use serde::{de::DeserializeOwned, Deserialize, Serialize};

use std::{
    clone::Clone, 
    cmp::PartialEq,
    fmt::{self, Debug, Display}
};

use nalgebra::{
    base::{Vector3, Vector4, Matrix6, Scalar},
    geometry::{Point3, Quaternion}
};

use crate::errors::RosError;

use std::fs::File;
use std::io::{self, Read};
use std::sync::Arc;
use std::str::FromStr;

use num::Num;
use num::Float;

use log::{debug, error, info, warn, trace};


pub trait RosMsg: Sized + Default{
    fn echo(&self) -> String;
    fn get_header(&self) -> Result<&Header, RosError>;
    fn from_vec(data: Vec<&str>) -> Result<Self, RosError>;
}

pub trait GeometryMsg: RosMsg + Serialize + DeserializeOwned + Display + Debug{}

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Header{
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>
}

impl RosMsg for Header{
    fn from_vec(data:  Vec<&str>) -> Result<Header, RosError>{

        let seq = match data[0].parse::<u32>(){
            Ok(result) => result,
            Err(e) => {
                println!("{e:}");
                return Err(RosError::ParseError{value: data[0].into(), name: "Header.seq".into()})
            }
        };

        let stamp = match data[1].parse::<i64>(){
            Ok(result) => result,
            Err(e) => return Err(RosError::ParseError{value: data[1].into(), name: "Header.timestamp".into()})
        };

        let dt = DateTime::from_timestamp_nanos(stamp);

        Ok(Header{
            seq: seq,
            time: dt,
            frame_id: Some(data[2].to_string())
        })
    }
    fn echo(&self) -> String{
        todo!()
    }
    fn get_header(&self) -> Result<&Header, RosError>{ Ok(&self) }
    
}



#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Pose
{
    position: Point3<f64>,
    orientation: Quaternion<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}
impl Pose
{
    fn parse_position(data:  &[Arc<str>]) -> Result<Point3<f64>, RosError>
    {
        //TODO: Check if this parser is working
        let vec = Vector3::from_iterator(
            data.iter()
                .map(|s| {
                    s.parse::<f64>().map_err(|_e| RosError::ParseError {
                        value: s.to_string().into(),
                        name: "Pose.position".into(),
                    })
                })
            .collect::<Result<Vec<f64>, RosError>>()?
        );

        return Ok(Point3::from(vec));
    }
    fn parse_orientation(data:  &[Arc<str>]) -> Result<Quaternion<f64>, RosError>
    {
        //TODO: Check if this parser is working
        let vec = Vector4::from_iterator(
            data.iter()
                .map(|s| {
                    s.parse::<f64>().map_err(|_e| RosError::ParseError {
                        value: s.to_string().into(),
                        name: "Pose.orientation".into(),
                    })
                })
            .collect::<Result<Vec<f64>, RosError>>()?
        );

        return Ok(Quaternion::from(vec));
    }

    fn parse_covariances(data:  &[Arc<str>]) -> Result<Matrix6<f64>, RosError>
     {
        //TODO: Check if this parser is working
        //I might want to ensure that the w is between -1 and 1;
        if data.len() != 36 {
            return Err(RosError::ParseError{value: "".into(), name: "Pose.covariance".into()})
        }

        let matrix = Matrix6::from_iterator(
            data.iter()
                .map(|s| {
                    s.parse::<f64>().map_err(|_e| RosError::ParseError {
                        value: s.to_string().into(),
                        name: "Pose.covariance".into(),
                    })
                })
            .collect::<Result<Vec<f64>, RosError>>()?
        );

        if matrix.sum() == 0.0 {
            return Err(RosError::ParseError{value: "".into(), name: "Pose.covariance".into()})
        }
        else{
            return Ok(matrix);
        };
    }   
}

impl RosMsg for Pose{
    fn echo(&self) -> String{
        todo!()
    }
    fn get_header(&self) -> Result<&Header, RosError>{
        Err(RosError::MissingHeader { rostype: "Pose".into() })
    }
    
    fn from_vec(data:  Vec<&str>) -> Result<Pose, RosError>{
        let data_vec: Vec<Arc<str>> = data.iter().map(|s| Arc::from(*s)).collect();

        let position: Point3<f64> = Self::parse_position(&data_vec[0..3])?;

        let quaternion: Quaternion<f64> = Self::parse_orientation(&data_vec[3..7])?;

        let pose = match Self::parse_covariances(&data_vec[7..data_vec.len()]){
            Ok(cov) => Pose{
                position: position,
                orientation: quaternion,
                covariance: Some(cov)
            },
            Err(e) => Pose{
                position: position,
                orientation: quaternion,
                covariance: None
            }

        };
        return Ok(pose);
    }

}
impl GeometryMsg for Pose{}

impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The odometry messages will be printed in the TUM format file
        write!(f, "{} {} {} {} {} {} {}",
            self.position[0], 
            self.position[1], 
            self.position[2], 
            self.orientation.coords[0], 
            self.orientation.coords[1], 
            self.orientation.coords[2], 
            self.orientation.coords[3]
        )
    }
}

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct PoseStamped
{
    header: Header,
    pose: Pose
}
impl RosMsg for PoseStamped{
    fn echo(&self) -> String{
        todo!()
    }
    fn get_header(&self) -> Result<&Header, RosError>{
        Ok(&self.header)
    }
    fn from_vec(data: Vec<&str>) -> Result<PoseStamped, RosError>{


        let header_data = vec!["0", data[0], "", ""];
        let header: Header = Header::from_vec(header_data)?;
        let pose: Pose = Pose::from_vec(data[4..data.len()].into())?;


        return Ok(PoseStamped{
            header,
            pose
        });
    }
}
impl GeometryMsg for PoseStamped{}

impl fmt::Display for PoseStamped {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The odometry messages will be printed in the TUM format file
        write!(f, "{} {} {} {} {} {} {} {}",
            self.header.time.timestamp_nanos_opt().unwrap() as f64 /f64::powf(10.0,9.0), 
            self.pose.position[0], 
            self.pose.position[1], 
            self.pose.position[2], 
            self.pose.orientation.coords[0], 
            self.pose.orientation.coords[1], 
            self.pose.orientation.coords[2], 
            self.pose.orientation.coords[3]
        )
    }
}



#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Twist
{
    linear: Vector3<f64>,
    angular: Vector3<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}
impl Twist
{
    fn parse_vector(data: &[Arc<str>]) -> Result<Vector3<f64>, RosError>
    {

        let vec = Vector3::from_iterator(
            data.iter()
                .map(|s| {
                    s.parse::<f64>().map_err(|_e| RosError::ParseError {
                        value: s.to_string().into(),
                        name: "Pose.position".into(),
                    })
                })
            .collect::<Result<Vec<f64>, RosError>>()?
        );

        return Ok(vec);
    }
    fn parse_covariances(data: &[Arc<str>]) -> Result<Matrix6<f64>, RosError>
     {

        let matrix = Matrix6::from_iterator(
            data.iter()
                .map(|s| {
                    s.parse::<f64>().map_err(|_e| RosError::ParseError {
                        value: s.to_string().into(),
                        name: "Pose.covariance".into(),
                    })
                })
            .collect::<Result<Vec<f64>, RosError>>()?
        );

        if matrix.sum() == 0.0 || data.len() != 36 {
            return Err(RosError::ParseError{value: "".into(), name: "Pose.covariance".into()})
        }
        else{
            return Ok(matrix);
        };
    }
}


impl RosMsg for Twist {
    fn echo(&self) -> String{
        todo!()
    }
    fn get_header(&self) -> Result<&Header, RosError>{
        Err(RosError::MissingHeader { rostype: "Pose".into() })
    }
    fn from_vec(data:  Vec<&str>) -> Result<Self, RosError> {
        let data_vec: Vec<Arc<str>> = data.iter().map(|s| Arc::from(*s)).collect();


        let linear = Self::parse_vector(&data_vec[0..3])?;
        let angular = Self::parse_vector(&data_vec[3..6])?;

        let twist = match Self::parse_covariances(&data_vec[6..data_vec.len()]){
            Ok(cov) => Twist{
                linear: linear,
                angular: angular,
                covariance: Some(cov)
            },
            Err(e) => Twist{
                linear: linear,
                angular: angular,
                covariance: None
            }
        };

        return Ok(twist);
    }

}


#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Odometry
{
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Pose,
    pub twist: Twist
}

impl RosMsg for Odometry{
    fn echo(&self) -> String{
        todo!()
    }
    fn get_header(&self) -> Result<&Header, RosError>{
        Ok(&self.header)
    }
    fn from_vec(data:  Vec<&str>) -> Result<Self, RosError> {

        if data.len() <= 48 {
            return Err(RosError::FormatError { name: format!("{:?}", data).into() })
        }

        let header: Header = Header::from_vec(data[1..4].into())?;
        let child_frame_id: &str = data[4];

        let pose: Pose = Pose::from_vec(data[5..48].into())?;
        let twist: Twist = Twist::from_vec(data[48..].into())?;


        Ok(Odometry{
            header: header,
            child_frame_id: None, //Ok(child_frame_id),
            pose: pose,
            twist: twist
        })
    }
}

impl GeometryMsg for Odometry { }

impl fmt::Display for Odometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The odometry messages will be printed in the TUM format file
        write!(f, "{} {} {} {} {} {} {} {}",
            self.header.time.timestamp_nanos_opt().unwrap() as f64 /f64::powf(10.0,9.0), 
            self.pose.position[0], 
            self.pose.position[1], 
            self.pose.position[2], 
            self.pose.orientation.coords[0], 
            self.pose.orientation.coords[1], 
            self.pose.orientation.coords[2], 
            self.pose.orientation.coords[3]
        )
    }
}