use std::sync::Arc;

use futures_util::future::ok;
use log::{info, warn};
// db/iteration.rs
use surrealdb::{Surreal, engine::local::Db, sql::Thing};
use tokio::sync::Mutex;
use crate::{models::{iteration::Iteration, Algorithm, AlgorithmRun, Dataset, Odometry, TestDefinition, TestExecution}, services::DbError};

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

    pub async fn get_dataset_thing(
        &self,
        iteration: &Iteration
    ) -> Result<Thing, DbError> {
        
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
                ))?.id.ok_or_else(|| DbError::MissingField(
                    "Missing ID for dataset"))
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


    pub async fn get_algorithm_run_thing(
        &self,
        iteration: &Iteration
    ) -> Result<Thing, DbError> {
        
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
                    format!("Algorithm Run for iteration {} not found", iteration_id)
                ))?.id.ok_or_else(|| DbError::MissingField(
                    "Missing ID for algorithm run"))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", iteration_id)
            ))
        }

    }



    pub async fn get_test_execution_thing(
        &self,
        iteration: &Iteration
    ) -> Result<Thing, DbError> {
        
        let iteration_id = iteration.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_iteration<-algorithm_run<-has_run<-test_execution.* AS test_execution
                FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

        let algorithm_run: Option<Vec<TestExecution>> = result.take("test_execution")?;

        match algorithm_run{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Test Execution for iteration {} not found", iteration_id)
                ))?.id.ok_or_else(|| DbError::MissingField(
                    "Missing ID for test execution"))
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

    pub async fn get_odometries(&self, iter: &Iteration) -> Result<Vec<Odometry>, DbError>{


        let iteration_id = iter.id.clone()
        .ok_or(DbError::MissingField("Iteration ID"))?;


        let mut result = self.conn.lock().await
            .query("
                    SELECT 
                        (SELECT * FROM $iteration_id->has_odometry->odometry.* ORDER BY header.time ASC) AS odometries
                    FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await?;

        // Handle nested array structure from graph query
        let nested_odoms: Vec<Vec<Odometry>> = result.take("odometries")?;
    
        // Flatten the results
        let odometries = nested_odoms
            .into_iter()
            .flatten()
            .collect::<Vec<Odometry>>();

        if odometries.is_empty() {
            Err(DbError::NotFound(
                format!("No odometry data found for iteration {}", iteration_id)
            ))
        } else {
            Ok(odometries)
        }
    }

    pub async fn get_odom_frequency(&self, iter: &Iteration) -> Result<f64, DbError> {

        let iteration_id = iter.id.clone()
        .ok_or(DbError::MissingField("Iteration ID"))?;


        let mut result_max = self.conn.lock().await
            .query("
                    SELECT 
                        (SELECT * FROM $iteration_id->has_odometry->odometry ORDER BY created_at DESC LIMIT 1) AS odometry
                    FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await.unwrap();

        let mut result_min = self.conn.lock().await
            .query("
                    SELECT 
                        (SELECT * FROM $iteration_id->has_odometry->odometry ORDER BY created_at ASC LIMIT 1) AS odometry
                    FROM $iteration_id
            ")
            .bind(("iteration_id", iteration_id.clone()))
            .await.unwrap();




        // Handle nested array structure from graph query
        let nested_odoms_min: Vec<Vec<Odometry>> = result_min.take("odometry").unwrap();
        let nested_odoms_max: Vec<Vec<Odometry>> = result_max.take("odometry").unwrap();

        let count = self.get_count_odoms(iter).await.unwrap();

        //warn!("My min odometry: {:?} \n My max odometry: {:?} \n My count: {count}", odoms_min, odoms_max);

    
        // Flatten the results
        let odom_min = nested_odoms_min
            .into_iter()
            .flatten()
            .next().ok_or(DbError::NotFound("Missing odometry value".to_owned()))?;

        let odom_max = nested_odoms_max
            .into_iter()
            .flatten()
            .next().ok_or(DbError::NotFound("Missing odometry value".to_owned()))?;

        let duration = odom_max.header.time - odom_min.header.time;

        let frequency = count as f64 / duration.num_seconds() as f64;

        warn!("My frequency of messages is: {frequency}");

        Ok(frequency)

    }

    async fn get_count_odoms(&self, iter: &Iteration) -> Result<u64, DbError>{

        let iteration_id = iter.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;


            let mut count_result = self.conn.lock().await
                .query("
                    SELECT count() AS total 
                    FROM $iteration_id->has_odometry->odometry
                    GROUP ALL
                ")
                .bind(("iteration_id", iteration_id))
                .await.unwrap();
            let count: Option<u64> = count_result.take("total").unwrap();

            count.ok_or(DbError::NotFound("Unable to get number of odometries".to_owned()))

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