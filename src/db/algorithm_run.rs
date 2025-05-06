use std::sync::Arc;

use log::warn;
use serde_json::Value;
// db/algorithm_run.rs
use surrealdb::{engine::local::Db, sql::Thing, Response, Surreal};
use tokio::sync::Mutex;

use crate::{models::{metric::Metric, metrics::pose_error::APE, Algorithm, AlgorithmRun, Iteration, TestDefinition, TestExecution}, services::DbError};

pub struct AlgorithmRunRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl AlgorithmRunRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(
        &self,
        run: &mut AlgorithmRun,
        test_execution_id: &Thing,
        algorithm_id: &Thing,
    ) -> Result<(), DbError> {

        //If slow add a Arc in here!
        
        let created: Option<AlgorithmRun> = self.conn.lock().await
            .create("algorithm_run")
            .content(run.clone())
            .await?;

        if let Some(created) = created {
            run.id = created.id;
        }

        let run_id = run.id.clone()
            .ok_or(DbError::NotFound("AlgorithmRun ID not found after creation".into()))?;

        // Create relationships
        self.conn.lock().await
            .query("RELATE $test_execution -> has_run -> $run")
            .bind(("test_execution", test_execution_id.clone()))
            .bind(("run", run_id.clone()))
            .await?;

        self.conn.lock().await
            .query("RELATE $run -> uses -> $algorithm")
            .bind(("run", run_id.clone()))
            .bind(("algorithm", algorithm_id.clone()))
            .await?;

        Ok(())
    }

    pub async fn get_algorithm(
        &self,
        run: &AlgorithmRun
    ) -> Result<Algorithm, DbError> {
        let run_id = run.id.clone()
            .ok_or(DbError::MissingField("AlgorithmRun ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT ->uses->algorithm.* AS algorithm 
                FROM $run_id
            ")
            .bind(("run_id", run_id.clone()))
            .await?;

        let algorithms: Option<Vec<Algorithm>> = result.take("algorithm")?;

        match algorithms{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Algorithm for run {} not found", run_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Algorithm for run {} not found", run_id)
            ))
        }

    }


    pub async fn get_test_definition(
        &self,
        algo_run: &AlgorithmRun
    ) -> Result<TestDefinition, DbError>{

            let algo_run_id = algo_run.id.clone()
            .ok_or(DbError::MissingField("Algorithm Run ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_run<-test_execution<-defines<-test_definition.* AS test_definition
                FROM $algo_run_id
            ")
            .bind(("algo_run_id", algo_run_id.clone()))
            .await?;


        let algorithm_run: Option<Vec<TestDefinition>> = result.take("test_definition")?;

        match algorithm_run{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Test Definition for algo_run {} not found", algo_run_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Test Definition for algo_run {} not found", algo_run_id)
            ))
        }
    }


    pub async fn get_test_execution_thing(
        &self,
        algo_run: &AlgorithmRun
    ) -> Result<Thing, DbError> {
        
        let algo_run_id = algo_run.id.clone()
            .ok_or(DbError::MissingField("Iteration ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT <-has_run<-test_execution.* AS test_execution
                FROM $algo_run_id
            ")
            .bind(("algo_run_id", algo_run_id.clone()))
            .await?;

        let algorithm_run: Option<Vec<TestExecution>> = result.take("test_execution")?;

        match algorithm_run{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Test Execution for iteration {} not found", algo_run_id)
                ))?.id.ok_or_else(|| DbError::MissingField(
                    "Missing ID for test execution"))
            },
            None => Err(DbError::NotFound(
                format!("Dataset for iteration {} not found", algo_run_id)
            ))
        }

    }

    pub async fn get_metrics(&self, run: &AlgorithmRun) -> Result<Vec<Metric>, DbError> {
        let run_id = run.id.clone()
            .ok_or(DbError::MissingField("AlgorithmRun ID"))?;
    
        let mut result = self.conn.lock().await
            .query("
                SELECT *
                FROM $run_id->has_iteration->iteration->has_metric->metric
            ")
            .bind(("run_id", run_id.clone()))
            .await?;
    
        let metrics: Vec<Metric> = result.take(0)?;
    
        Ok(metrics)
    }

    pub async fn get_iterations(&self, run: &AlgorithmRun) -> Result<Vec<Iteration>, DbError> {
        let run_id = run.id.clone()
            .ok_or(DbError::MissingField("AlgorithmRun ID"))?;
    
        let mut result = self.conn.lock().await
            .query("
                SELECT *
                FROM $run_id->has_iteration->iteration
            ")
            .bind(("run_id", run_id.clone()))
            .await?;
    
        let metrics: Vec<Iteration> = result.take(0)?;
    
        Ok(metrics)
    }


    pub async fn update_aggregate_metric(&self, run: &AlgorithmRun, metric: Metric) -> Result<(), DbError> {

        let run_id = run.id.clone()
        .ok_or(DbError::MissingField("AlgorithmRun ID"))?;

        let mut result = self.conn.lock().await
            .query("
                UPDATE $run_id SET metrics += $metric
            ")
            .bind(("run_id", run_id.clone()))
            .bind(("metric", metric))
            .await?;


        Ok(())

    }


}



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


