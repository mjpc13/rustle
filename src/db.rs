use anyhow::{anyhow, Result};

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

use surrealdb::{
    sql::{Thing, Array, Object, Value, to_value},
    Response,
    Surreal,
    engine::any::Any
};

use bollard::container::{MemoryStats, CPUStats};
use crate::ros_msgs::{Pose, Twist, Header, Odometry};

use log::{info};

// =====DATA MODELS=====
#[derive(Debug, Serialize, Deserialize)]
pub struct Stats {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub memory_stats: MemoryStats,
    pub cpu_stats: CPUStats,
    pub num_procs: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Metrics{
    #[serde(skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    pub ape: u32,
    pub rte: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

#[derive(Debug, Serialize, Deserialize)]
struct Record{
    #[allow(dead_code)]
    id: Thing
}
// =====DATA MODELS=====



#[derive(Clone)]
pub struct DB{
    pub db: Surreal<Any>,
}
impl DB{
    pub async fn add_stat(&self, mut data: Stats){
        
        data.created_at = Some(Utc::now());

        let create: Vec<Record> = self.db.create("stat")
            .content(data)
            .await.unwrap();

        let stats: Vec<Record> = self.db.select("stat").await.unwrap();

    }

    pub async fn query_stats(&self) -> Response {
        //TODO: return an array of Stats instead of the Response
        //TODO: Probably add a more relevant query

        let groups = self.db
            .query("SELECT * FROM type::table($table)")
            .bind(("table", "stat"))
            .await.unwrap();
        groups
    }
    pub async fn add_odom(&self, mut data: Odometry, table: &str)
    {
        let create: Vec<Record> = self.db.create(table)
            .content(data)
            .await
            .unwrap();
    }

    pub async fn query_odom(&self, table: &str) -> Result<Vec<Odometry>, surrealdb::Error> 
    {    
        let mut response = self.db
            .query("SELECT * FROM type::table($table)")
            .bind(("table", table))
            .await.unwrap();

        let odoms: Vec<_> = response.take(0)?;
        
        Ok(odoms)
    }
}
