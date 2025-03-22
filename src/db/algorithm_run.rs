use std::sync::Arc;

use surrealdb::{Surreal, engine::remote::ws::Client};
use tokio::sync::Mutex;
use crate::{models::algorithm_run::AlgorithmRun, services::error::DbError};

pub struct AlgorithmRunRepo {
    conn: Arc<Mutex<Surreal<Client>>>,
}

impl AlgorithmRunRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Client>>>) -> Self {
        Self { conn }
    }

    //pub async fn save(&self, run: &AlgorithmRun) -> Result<(), DbError> {
    //    self.conn
    //        .create(("algorithm_run", &run.id))
    //        .content(run)
    //        .await
    //        .map_err(|e| DbError::Operation(e))
    //}

    //pub async fn get_for_test_execution(
    //    &self,
    //    test_execution_id: &str
    //) -> Result<Vec<AlgorithmRun>, DbError> {
    //    self.conn
    //        .query("SELECT * FROM algorithm_run WHERE test_execution_id = $test_execution_id")
    //        .bind(("test_execution_id", test_execution_id))
    //        .await?
    //        .take(0)
    //        .map_err(|e| DbError::Operation(e))
    //}

    //pub async fn get_for_algorithm(
    //    &self,
    //    algorithm_id: &str
    //) -> Result<Vec<AlgorithmRun>, DbError> {
    //    self.conn
    //        .query("SELECT * FROM algorithm_run WHERE algorithm_id = $algorithm_id")
    //        .bind(("algorithm_id", algorithm_id))
    //        .await?
    //        .take(0)
    //        .map_err(|e| DbError::Operation(e))
    //}
}