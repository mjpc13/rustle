use std::{fmt::format, sync::Arc};

use log::{info, warn};
use surrealdb::{engine::local::Db, Surreal};
use tokio::sync::Mutex;
use crate::{models::{test_execution::TestExecution, Algorithm, Dataset, TestDefinition}, services::error::DbError};
use surrealdb::sql::Thing;

pub struct TestExecutionRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl TestExecutionRepo {
    pub fn new(  conn: Arc<Mutex<Surreal<Db>>> ) -> Self {
        Self { conn }
    }

    pub async fn save(&self, execution: &mut TestExecution, def: &TestDefinition) -> Result<(), DbError> {

        let created: Option<TestExecution> = self.conn.lock().await
            .create("test_execution")
            .content(execution.clone())
            .await?;

        info!("The execution record was CREATED!!!!");

           if let Some(created) = created {
                execution.id = created.id;
            }

            // Validate and get the execution ID
            let execution_id = execution.id.clone()
                .ok_or(DbError::NotFound("TestExecution ID not found after creation".into()))?;

            let test_execution = format!("test_execution:{}", execution_id.clone());

            // Create relationship with TestDefinition
            let rel = self.conn.lock().await
                .query("RELATE $def -> defines -> $test_execution")
                .bind(("test_execution", execution_id.clone()))
                .bind(("def", def.id.clone().unwrap()))
                .await.unwrap();


            // Create Dataset relationship
            let dataset = self.get_dataset_by_name(def.dataset_name.clone()).await.unwrap();
            // Create Dataset relationship
            let rel = self.conn.lock().await
                .query("RELATE $test_execution -> tested_in -> $dataset")
                .bind(("test_execution", execution_id.clone()))
                .bind(("dataset", dataset.id.clone()))
                .await.unwrap();

            for algo in def.algo_list.clone() {
                // Create Dataset relationship
                let algo = self.get_algorithm_by_name(algo).await.unwrap();

                // Create Dataset relationship
                let rel = self.conn.lock().await
                    .query("RELATE $test_execution -> compares -> $algo")
                    .bind(("test_execution", execution_id.clone()))
                    .bind(("algo", algo.id.clone()))
                    .await.unwrap();
            }

        Ok(())  
    }

    pub async fn get(&self, id: &str) -> Result<Option<TestExecution>, DbError> {
        self.conn.lock().await
            .select(("test_execution", id))
            .await
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn list_active(&self) -> Result<Vec<TestExecution>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM test_execution WHERE status IN ['Scheduled', 'Running']")
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn get_dataset_by_name(&self, db_name: String) -> Result<Dataset, DbError> {
        let mut response = self.conn.lock().await
            .query("SELECT * FROM dataset WHERE name = $name LIMIT 1")
            .bind(("name", db_name.clone()))
            .await?;
    
        let dataset_id: Option<Dataset> = response.take(0)?;
        
        dataset_id.ok_or_else(|| DbError::NotFound(
            format!("Dataset with name '{}' not found", db_name)
        ))
    }

    pub async fn get_algorithm_by_name(&self, db_name: String) -> Result<Algorithm, DbError> {
        let mut response = self.conn.lock().await
            .query("SELECT * FROM algorithm WHERE name = $name LIMIT 1")
            .bind(("name", db_name.clone()))
            .await?;
    
        let dataset_id: Option<Algorithm> = response.take(0)?;
        
        dataset_id.ok_or_else(|| DbError::NotFound(
            format!("Algorithm with name '{}' not found", db_name)
        ))
    }

    pub async fn get_def(&self, execution: &TestExecution) -> Result<TestDefinition, DbError> {
        
        let execution_id = execution.id.clone()
            .ok_or(DbError::MissingField("TestExecution ID"))?;

        let mut result = self.conn.lock().await
            .query("SELECT <-defines<-test_definition AS definition FROM $exec_id")
            .bind(("exec_id", execution_id.clone()))
            .await?;

        let definition: Option<TestDefinition> = result.take("definition")?;
        definition.ok_or_else(|| DbError::NotFound(
            format!("TestDefinition for execution {}", execution_id)
        ))
    }

    pub async fn get_dataset(&self, execution: &TestExecution) -> Result<Dataset, DbError> {
        let execution_id = execution.id.clone()
            .ok_or(DbError::MissingField("TestExecution ID"))?;

        let mut result = self.conn.lock().await
            .query("SELECT ->tested_in->dataset AS dataset FROM $exec_id")
            .bind(("exec_id", execution_id))
            .await?;

        let dataset: Option<Dataset> = result.take("dataset")?;

        
        Ok(dataset.unwrap())
    }

    pub async fn get_algos(&self, execution: &TestExecution) -> Result<Vec<Algorithm>, DbError> {
        
        let execution_id = execution.id.clone()
            .ok_or(DbError::MissingField("TestExecution ID"))?;

        let mut result = self.conn.lock().await
            .query("SELECT ->compares->algorithm AS algorithms FROM $exec_id")
            .bind(("exec_id", execution_id.clone()))
            .await?;

        let algorithms: Vec<Algorithm> = result.take("algorithms")?;
        if algorithms.is_empty() {
            Err(DbError::NotFound(
                format!("Algorithms for execution {}", execution_id)
            ))
        } else {
            Ok(algorithms)
        }
    }


}