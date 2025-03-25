use std::sync::Arc;

use log::{info, warn};
// db/iteration.rs
use surrealdb::{Surreal, engine::local::Db, sql::Thing};
use tokio::sync::Mutex;
use crate::{models::{iteration::Iteration, Algorithm, AlgorithmRun, Dataset, TestDefinition}, services::DbError};

#[derive(Clone)]
pub struct IterationRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl IterationRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, iteration: &mut Iteration, algorithm_run_id: &Thing) -> Result<(), DbError> {

        //If slow add a Arc in here!
        let created: Option<Iteration> = self.conn.lock().await
            .create("iteration")
            .content(iteration.clone())
            .await?;

        if let Some(created) = created {
            iteration.id = created.id;
        }

        match iteration.id.clone(){
            Some(id) => {
                // Create relationships
                let t = self.conn.lock().await
                    .query("RELATE $algorithm_run -> has_iteration -> $iteration")
                    .bind(("algorithm_run", algorithm_run_id.clone()))
                    .bind(("iteration", id))
                    .await?;
            }
            None => warn!("Iteration does not have an ID")
        };

        Ok(())
    }

    pub async fn get_dataset(
        &self,
        iteration: &Iteration
    ) -> Result<Dataset, DbError> {
        
        let iteration_id = iteration.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_iteration<-algorithm_run<-has_run<-test_execution->tested_in->dataset.* AS dataset
                FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

            //warn!("My response from db: {:?}", result);


        let dataset: Option<Vec<Dataset>> = result.take("dataset")?;

        match dataset{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Dataset for iteration {} not found", iteration_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", iteration_id)
            ))
        }

    }

    pub async fn get_algorithm(
        &self,
        iteration: &Iteration
    ) -> Result<Algorithm, DbError> {
        
        let iteration_id = iteration.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_iteration<-algorithm_run->uses->algorithm.* AS algorithm
                FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

        let algorithm: Option<Vec<Algorithm>> = result.take("algorithm")?;

        match algorithm{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Dataset for iteration {} not found", iteration_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", iteration_id)
            ))
        }

    }

    pub async fn get_algorithm_run(
        &self,
        iteration: &Iteration
    ) -> Result<AlgorithmRun, DbError> {
        
        let iteration_id = iteration.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_iteration<-algorithm_run.* AS algorithm_run
                FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

        let algorithm_run: Option<Vec<AlgorithmRun>> = result.take("algorithm_run")?;

        match algorithm_run{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Dataset for iteration {} not found", iteration_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", iteration_id)
            ))
        }

    }

    pub async fn get_test_def(&self, iteration: &Iteration) -> Result<TestDefinition, DbError>{

        let iteration_id = iteration.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_iteration<-algorithm_run<-has_run<-test_execution<-defines<-test_definition.* AS test_definition
                FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

        

        let algorithm_run: Option<Vec<TestDefinition>> = result.take("test_definition")?;

        match algorithm_run{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Dataset for iteration {} not found", iteration_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", iteration_id)
            ))
        }


    }



    //pub async fn get_by_run(&self, run_id: &str) -> Result<Vec<Iteration>, DbError> {
    //    let mut result = self.conn
    //        .query("SELECT * FROM iteration WHERE run = $run_id")
    //        .bind(("run_id", run_id))
    //        .await?;

    //    let iterations: Vec<Iteration> = result.take(0)?;
    //    Ok(iterations)
    //}
}