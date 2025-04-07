use std::sync::Arc;

use surrealdb::{Surreal, engine::local::Db, sql::Thing};
use tokio::sync::Mutex;
use crate::{models::{ros::odometry::Odometry, Iteration}, services::DbError};

#[derive(Clone)]
pub struct OdometryRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl OdometryRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, odom: &mut Odometry, iteration_id: &Thing) -> Result<(), DbError> {


        //This should only happen 1 time
        self.conn.lock().await
            .query("DEFINE INDEX odom_created_at ON odom FIELDS created_at")
            .await?;

        let created: Option<Odometry> = self.conn.lock().await
            .create("odometry")
            .content(odom.clone())
            .await?;

        if let Some(created) = created {
            odom.id = created.id;
        }

        if let Some(id) = &odom.id {
            self.conn.lock().await
                .query("RELATE $iteration->has_odometry->$odom")
                .bind(("iteration", iteration_id.clone()))
                .bind(("odom", id.clone()))
                .await?;
        }

        Ok(())
    }

//    pub async fn get_by_iteration(&self, iteration_id: &Thing) -> Result<Vec<Odometry>, DbError> {
//        let mut result = self.conn
//            .query("SELECT * FROM odometry WHERE iteration_id = $iteration_id")
//            .bind(("iteration_id", iteration_id))
//            .await?;
//
//        let odometries: Vec<Odometry> = result.take(0)?;
//        Ok(odometries)
//    }
}