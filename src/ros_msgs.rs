use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use std::{
    clone::Clone, 
    cmp::PartialEq,
    fmt::Debug,
    fmt
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


pub trait RosMsg{}



#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Header{
    pub seq: u32,
    pub time: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame_id: Option<String>
}
impl Header
{
   pub fn parse(data: &[Arc<str>]) -> Result<Header, RosError>{

        let seq = match data[0].parse::<u32>(){
            Ok(result) => result,
            Err(e) => return Err(RosError::ParseError{value: data[0].clone().into(), name: "Header.seq".into()})
        };

        let stamp = match data[1].parse::<i64>(){
            Ok(result) => result,
            Err(e) => return Err(RosError::ParseError{value: data[1].clone().into(), name: "Header.timestamp".into()})
        };

        let dt = DateTime::from_timestamp_nanos(stamp);

        Ok(Header{
            seq: seq,
            time: dt,
            frame_id: Some(data[2].to_string())
        })
    }
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
    pub fn parse(data: &[Arc<str>]) -> Result<Pose, RosError>{
        let position: Point3<f64> = Self::parse_position(&data[0..3])?;
        let quaternion: Quaternion<f64> = Self::parse_orientation(&data[3..7])?;

        let pose = match Self::parse_covariances(&data[7..data.len()]){
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
    fn parse_position(data: &[Arc<str>]) -> Result<Point3<f64>, RosError>
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
    fn parse_orientation(data: &[Arc<str>]) -> Result<Quaternion<f64>, RosError>
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

    fn parse_covariances(data: &[Arc<str>]) -> Result<Matrix6<f64>, RosError>
     {
        //TODO: Check if this parser is working
        //I might want to ensure that the w is between -1 and 1;
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

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct PoseStamped
{
    header: Header,
    position: Point3<f64>,
    orientation: Quaternion<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    covariance: Option<Matrix6<f64>>
}
impl PoseStamped
{
    pub fn parse(data: Vec<&str>) -> Result<PoseStamped, RosError>{
        let data_vec: Vec<Arc<str>> = data.iter().map(|s| Arc::from(*s)).collect();

        let header: Header = Header::parse(&data_vec[1..4])?;
        let position: Point3<f64> = Self::parse_position(&data_vec[0..3])?;
        let quaternion: Quaternion<f64> = Self::parse_orientation(&data_vec[3..7])?;

        let pose = match Self::parse_covariances(&data_vec[7..data_vec.len()]){
            Ok(cov) => PoseStamped{
                header: header,
                position: position,
                orientation: quaternion,
                covariance: Some(cov)
            },
            Err(e) => PoseStamped{
                header: header,
                position: position,
                orientation: quaternion,
                covariance: None
            }

        };

        return Ok(pose);
    }
    fn parse_position(data: &[Arc<str>]) -> Result<Point3<f64>, RosError>
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
    fn parse_orientation(data: &[Arc<str>]) -> Result<Quaternion<f64>, RosError>
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

    fn parse_covariances(data: &[Arc<str>]) -> Result<Matrix6<f64>, RosError>
     {
        //TODO: Check if this parser is working
        //I might want to ensure that the w is between -1 and 1;
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
   pub fn parse(data: &[Arc<str>]) -> Result<Twist, RosError>{

        let linear = Self::parse_vector(&data[0..3])?;
        let angular = Self::parse_vector(&data[3..6])?;

        let twist = match Self::parse_covariances(&data[6..data.len()]){
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

    fn parse_vector(data: &[Arc<str>]) -> Result<Vector3<f64>, RosError>
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

        return Ok(vec);
    }
    fn parse_covariances(data: &[Arc<str>]) -> Result<Matrix6<f64>, RosError>
     {
        //TODO: Check if this parser is working
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

#[derive(Debug, Serialize, Deserialize, Default)]
pub struct Odometry
{
    pub header: Header,
    pub child_frame_id: Option<String>,
    pub pose: Pose,
    pub twist: Twist
}

impl Odometry
{
   pub fn parse(&self, data: Vec<&str>) -> Result<Odometry, RosError>{

        let data_vec: Vec<Arc<str>> = data.iter().map(|s| Arc::from(*s)).collect();

        let header: Header = Header::parse(&data_vec[1..4])?;
        let child_frame_id: &Arc<str> = &data_vec[4];
        let pose: Pose = Pose::parse(&data_vec[5..48])?;
        let twist: Twist = Twist::parse(&data_vec[48..])?;

        Ok(Odometry{
            header: header,
            child_frame_id: None, //Some(child_frame_id),
            pose: pose,
            twist: twist
        })
    }
}

impl fmt::Display for Odometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The odometry messages will be printed in the TUM format file
        write!(f, "{}, {}, {}, {}, {}, {}, {}, {}",
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

impl RosMsg for Odometry { }
impl RosMsg for PoseStamped { }
