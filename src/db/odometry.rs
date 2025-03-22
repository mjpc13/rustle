use surrealdb::{Surreal, engine::remote::ws::Client};
use crate::{models::ros::Odometry, services::error::DbError};

pub struct OdometryRepo {
    conn: Surreal<Client>,
}

impl OdometryRepo {
    pub fn new(conn: Surreal<Client>) -> Self {
        Self { conn }
    }

    //pub async fn save_batch(
    //    &self,
    //    odometries: &[Odometry]
    //) -> Result<(), DbError> {
    //    self.conn
    //        .query("INSERT INTO odometry $odometries")
    //        .bind(("odometries", odometries))
    //        .await
    //        .map_err(|e| DbError::Operation(e))?;
    //    Ok(())
    //}

    //pub async fn get_for_test_run(
    //    &self,
    //    test_run_id: &str
    //) -> Result<Vec<Odometry>, DbError> {
    //    self.conn
    //        .query("SELECT * FROM odometry 
    //                WHERE test_run_id = $test_run_id
    //                ORDER BY header.time ASC")
    //        .bind(("test_run_id", test_run_id))
    //        .await?
    //        .take(0)
    //        .map_err(|e| DbError::Operation(e))
    //}
}