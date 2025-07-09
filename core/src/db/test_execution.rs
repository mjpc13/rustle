use std::sync::Arc;

use surrealdb::{engine::local::Db, RecordId, Surreal};
use tokio::sync::Mutex;
use crate::{models::{test_execution::TestExecution, Algorithm, AlgorithmRun, Dataset, Iteration, TestDefinition}, services::error::DbError};
use surrealdb::sql::Thing;

pub struct TestExecutionRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl TestExecutionRepo {
    pub fn new(  conn: Arc<Mutex<Surreal<Db>>> ) -> Self {
        Self { conn }
    }

    pub async fn save(&self, execution: &mut TestExecution) -> Result<(), DbError> {

        let created: Option<TestExecution> = self.conn.lock().await
            .create("test_execution")
            .content(execution.clone())
            .await?;

           if let Some(created) = created {
                execution.id = created.id;
            }

            // Validate and get the execution ID
            let execution_id = execution.id.clone()
                .ok_or(DbError::NotFound("TestExecution ID not found after creation".into()))?;

            // Create Dataset relationship
            let dataset = self.get_dataset_by_name(&execution.def.dataset_name.clone()).await.unwrap();
            // Create Dataset relationship
            self.conn.lock().await
                .query("RELATE $test_execution -> tested_in -> $dataset")
                .bind(("test_execution", execution_id.clone()))
                .bind(("dataset", dataset.id.clone()))
                .await.unwrap();

            for algo in execution.def.algo_list.clone() {
                // Create Dataset relationship
                let algo = self.get_algorithm_by_name(algo).await.unwrap();

                // Create Dataset relationship
                self.conn.lock().await
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

    pub async fn get_by_name(&self, name: String) -> Result<Option<TestExecution>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM test_execution WHERE def.name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn list_active(&self) -> Result<Vec<TestExecution>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM test_execution WHERE status IN ['Scheduled', 'Running']")
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn get_dataset_by_name(&self, db_name: &String) -> Result<Dataset, DbError> {
        let mut response = self.conn.lock().await
            .query("SELECT * FROM dataset WHERE name = $name LIMIT 1")
            .bind(("name", db_name.clone()))
            .await?;
    
        let dataset_id: Option<Dataset> = response.take(0)?;
        
        dataset_id.ok_or_else(|| DbError::NotFound(
            format!("Dataset with name '{}' not found", db_name)
        ))
    }
    
    pub async fn get_algorithm_by_name(&self, name: String) -> Result<Algorithm, DbError> {
        let mut response = self.conn.lock().await
            .query("SELECT * FROM algorithm WHERE name = $name LIMIT 1")
            .bind(("name", name.clone()))
            .await?;
    
        // First take the first result from the first query statement
        let algorithm: Option<Algorithm> = response.take(0)?; 
    
        algorithm.ok_or_else(|| DbError::NotFound(
            format!("Algorithm with name '{}' not found", name)
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
            .query("
                SELECT ->compares->algorithm.* AS algorithms 
                FROM $exec_id
            ")
            .bind(("exec_id", execution_id.clone()))
            .await?;

        // Get the first query result (index 0)
        let raw_algorithms: Vec<Vec<Algorithm>> = result.take("algorithms")?;

        // Flatten the nested results
        let algorithms: Vec<Algorithm> = raw_algorithms.into_iter().flatten().collect();

        if algorithms.is_empty() {
            Err(DbError::NotFound(
                format!("Algorithms for execution {}", execution_id)
            ))
        } else {
            Ok(algorithms)
        }
    }

    pub async fn get_iterations(
        &self,
        test_execution_id: &Thing,
    ) -> Result<Vec<Iteration>, DbError> {
        let mut result = self.conn.lock().await
            .query("
                SELECT ->has_run->algorithm_run->has_iteration->iteration.* AS iterations
                FROM $test_execution_id
            ")
            .bind(("test_execution_id", test_execution_id.clone()))
            .await?;

        // Handle nested array structure from graph traversal
        let nested_iterations: Vec<Vec<Iteration>> = result.take("iterations")?;

        
        // Flatten the results
        let iterations = nested_iterations
            .into_iter()
            .flatten()
            .collect::<Vec<Iteration>>();

        if iterations.is_empty() {
            Err(DbError::NotFound(format!(
                "No iterations found for test execution {}",
                test_execution_id
            )))
        } else {
            Ok(iterations)
        }
    }


    pub async fn get_algorithm_runs(
        &self,
        test_execution_id: &Thing,
    ) -> Result<Vec<AlgorithmRun>, DbError> {
        let mut result = self.conn.lock().await
            .query("
                SELECT ->has_run->algorithm_run.* AS algorithm_runs
                FROM $test_execution_id
            ")
            .bind(("test_execution_id", test_execution_id.clone()))
            .await?;

        // Handle nested array structure from graph traversal
        let nested_algorithm_run: Vec<Vec<AlgorithmRun>> = result.take("algorithm_runs")?;

        
        // Flatten the results
        let algorithm_run = nested_algorithm_run
            .into_iter()
            .flatten()
            .collect::<Vec<AlgorithmRun>>();

        if algorithm_run.is_empty() {
            Err(DbError::NotFound(format!(
                "No iterations found for test execution {}",
                test_execution_id
            )))
        } else {
            Ok(algorithm_run)
        }
    }


    pub async fn list_all(&self) -> Result<Vec<TestExecution>, surrealdb::Error> {
        self.conn.lock().await.query("SELECT * FROM test_execution").await?.take(0)
    }


    pub async fn delete_by_name(&self, name: String) -> Result<(), DbError> {
        self.conn.lock().await
            .query("DELETE FROM test_execution WHERE name = $name")
            .bind(("name", name))
            .await
            .map_err(DbError::Operation)?;
    
        Ok(())
    }


    pub async fn clean_exec(&self, test_exec: TestExecution) -> Result<(), DbError> {


        let test_id = test_exec.id
            .ok_or(DbError::MissingField("TestDefinition ID"))?;

        let mut delete_list: Vec<Thing> = Vec::new();


        //Delete algorithm_run
        self.conn.lock().await
            .query("DELETE algorithm_run WHERE exec_id =  $id" )
            .bind(("id", test_id.clone()))
            .await
            .map_err(DbError::Operation).unwrap();

        //Delete algorithm_run
        self.conn.lock().await
            .query("DELETE iteration WHERE exec_id =  $id" )
            .bind(("id", test_id))
            .await
            .map_err(DbError::Operation).unwrap();





        // let exec_id_list: Vec<Thing> = vec![];

        // delete_list.extend(exec_id_list.iter().cloned());

        // for exec_id in exec_id_list{

        //     // Get things for the test executions
        //     let mut result_algo = self.conn.lock().await
        //         .query("RETURN (
        //                   SELECT out FROM has_run WHERE in = $id
        //                 ).out;" )
        //         .bind(("id", exec_id))
        //         .await
        //         .map_err(DbError::Operation).unwrap();

        //     let algo_run_id_list: Vec<Thing> = result_algo.take(0).unwrap();

        //     delete_list.extend(algo_run_id_list.iter().cloned());


        //     for algo_run_id in algo_run_id_list{

        //                         // Get things for the test executions
        //         let mut result_iter = self.conn.lock().await
        //             .query("RETURN (
        //                       SELECT out FROM has_iteration WHERE in = $id
        //                     ).out;" )
        //             .bind(("id", algo_run_id))
        //             .await
        //             .map_err(DbError::Operation).unwrap();
                
        //         let iter_id_list: Vec<Thing> = result_iter.take(0).unwrap();
        //         delete_list.extend(iter_id_list.iter().cloned());

        //         for iter_id in iter_id_list {

        //             let iter_id = Arc::new(iter_id);

        //             //Get APEs
        //             let mut result_ape = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_ape WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut apes: Vec<Thing> = result_ape.take(0).unwrap();

        //             //Get RPEs
        //             let mut result_rpe = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_rpe WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut rpes: Vec<Thing> = result_rpe.take(0).unwrap();

        //             //Get Odometry
        //             let mut result_odom = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_odometry WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut odoms: Vec<Thing> = result_odom.take(0).unwrap();          

        //             //Get Position
        //             let mut result_pos = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_position WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut positions: Vec<Thing> = result_pos.take(0).unwrap();              
                    
        //             //Get Metric
        //             let mut result_metric = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_metric WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut metrics: Vec<Thing> = result_metric.take(0).unwrap();

        //             //Get Stats
        //             let mut result_stat = self.conn.lock().await
        //                 .query("RETURN (
        //                           SELECT out FROM has_stat WHERE in = $name
        //                         ).out;" )
        //                 .bind(("name", iter_id.clone()))
        //                 .await
        //                 .map_err(DbError::Operation).unwrap();
        //             let mut stats: Vec<Thing> = result_stat.take(0).unwrap();

        //             delete_list.append(&mut apes);
        //             delete_list.append(&mut rpes);
        //             delete_list.append(&mut stats);
        //             delete_list.append(&mut metrics);
        //             delete_list.append(&mut odoms);
        //             delete_list.append(&mut positions);

        //         }



        //     }


        // }

        // //Delete all items
        // self.conn.lock().await
        //     .query("DELETE $items" )
        //     .bind(("items", delete_list))
        //     .await
        //     .map_err(DbError::Operation).unwrap();

        Ok(())
    
    }


    pub async fn update_execution(&self, exec: &TestExecution) -> Result<(), DbError> {

        let exec_id = exec.id.clone()
        .ok_or(DbError::MissingField("Test Execution ID"))?;

        let _updated: TestExecution = self.conn.lock().await
            .update((&exec_id.tb, &exec_id.id.to_string()))
            .content(exec.clone())
            .await?.unwrap();


        Ok(())

    }

    pub async fn get_by_algo_dataset_type(&self, algo_name: &str, dataset_name: &str, test_type: &str) -> Result<Vec<TestExecution>, DbError>{

        let mut resp = self.conn.lock().await
            .query("SELECT * FROM test_execution WHERE $algo_name IN def.algo_list AND def.dataset_name = $dataset_name AND def.test_type.type = $test_type;")
            .bind(("algo_name", algo_name.to_string()))
            .bind(("dataset_name", dataset_name.to_string()))
            .bind(("test_type", test_type.to_string()))
            .await?;

        let vec: Vec<TestExecution> = resp.take(0)?;

        if vec.is_empty(){
            return Err(DbError::Empty("There is no Test Definitions with those parameters.".into()));
        }

        Ok(vec)
    }

}