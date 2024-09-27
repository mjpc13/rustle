use anyhow::{anyhow, Result};

use chrono::{DateTime, Utc};
use serde::{de, Deserialize, Serialize};
use std::{fmt::Debug, sync::{Arc}};

use tokio::sync::Mutex;

use surrealdb::{
    sql::{Thing, Array, Object, Value, to_value},
    Response,
    Surreal,
    engine::any::Any
};

use bollard::container::{MemoryStats, CPUStats};
use crate::{metrics::ContainerStats, ros_msgs::{Header, Odometry, Pose, RosMsg, Twist}};

use log::{debug, error, info, warn, trace};


#[derive(Debug, Serialize, Deserialize)]
struct Record{
    #[allow(dead_code)]
    id: Thing
}

#[derive(Debug, Clone)]
pub struct DB{
    pub db: Arc<Mutex<Surreal<Any>>>,
}
impl DB{
    pub async fn add_stat(&self, namespace: &str, task_id: &str, mut data: ContainerStats){

        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();
        
        data.created_at = Some(Utc::now());

        let create: Vec<Record> = db_lock
            .create("stat")
            .content(data)
            .await.unwrap();

        let stats: Vec<Record> =  db_lock.select("stat").await.unwrap();
    }

    pub async fn query_stats(&self,  namespace: &str, task_id: &str) -> Result<Vec<ContainerStats>, surrealdb::Error> {    

        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();


        let mut groups = db_lock
            .query("SELECT * FROM type::table($table) ORDER BY uid;")
            .bind(("table", "stat"))
            .await.unwrap();

        let stats: Vec<ContainerStats> = groups.take(0)?;

        debug!("Query of Stats: {:#?}", stats);
        Ok(stats)
    }

    pub async fn add_odom(&self, namespace: &str, task_id: &str, mut data: Odometry, table: &str)
    {
        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();

        let create: Vec<Record> = db_lock
            .create(table)
            .content(data)
            .await
            .unwrap();

        debug!("Odom added to DB: {:?}", create);
    }

    pub async fn query_odom(&self, namespace: &str, task_id: &str, table: &str) -> Result<Vec<Odometry>, surrealdb::Error>
    {    
        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();

        let mut response = db_lock
            .query("SELECT * FROM type::table($table) ORDER BY header")
            .bind(("table", table))
            .await?;

        let odoms: Vec<_> = response.take(0)?;

        Ok(odoms)
    }
}
