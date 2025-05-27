use core::error;
use std::sync::{Arc};

use log::{info, warn};
use surrealdb::{engine::local::Db, sql::Thing, Object, Surreal};
use tokio::sync::Mutex;
use crate::{models::{metrics::pose_error::APE, TestDefinition, TestExecution}, services::DbError};

#[derive(Debug, Clone)]
pub struct TestDefinitionRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl TestDefinitionRepo {

    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, def: &mut TestDefinition) -> Result<(), surrealdb::Error> {
        
        let created: Option<TestDefinition> = self.conn.lock().await
            .create("test_definition")
            .content(def.clone())
            .await?;

            if let Some(created) = created {
                def.id = created.id;
            }
        Ok(())
    }

    pub async fn get(&self, id: &str) -> Result<Option<TestDefinition>, surrealdb::Error> {
        self.conn.lock().await.select(("test_definition", id)).await
    }

    pub async fn get_by_name(&self, name: String) -> Result<Option<TestDefinition>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM test_definition WHERE name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn get_test_executions(&self, def: &TestDefinition) -> Result<TestExecution, DbError> {
        
        let definition_id = def.id.clone()
            .ok_or(DbError::MissingField("TestDefinition ID"))?;

        let mut result = self.conn.lock().await
            .query("SELECT ->defines->test_execution.* AS execution FROM $def_id")
            .bind(("def_id", definition_id.clone()))
            .await?;

        let executions: Option<Vec<TestExecution>> = result.take("execution").unwrap();
        
        let execution = executions
            .and_then(|mut vec| vec.pop()) // get the first if it exists
            .ok_or_else(|| DbError::NotFound(
                format!("TestDefinition for execution {}", definition_id)
            ))?;

        Ok(execution)
    }


    
    pub async fn list(
        &self
    ) -> Result<Vec<TestDefinition>, surrealdb::Error> {
        self.conn.lock().await
            .query("SELECT * FROM test_definition")
            .await?
            .take(0)
    }


    pub async fn list_by_type(
        &self, 
        test_type: String
    ) -> Result<Vec<TestDefinition>, surrealdb::Error> {
        self.conn.lock().await
            .query("SELECT * FROM test_definition WHERE test_type.type = $type")
            .bind(("type", test_type))
            .await?
            .take(0)
    }

    pub async fn list_all(&self) -> Result<Vec<TestDefinition>, surrealdb::Error> {
        self.conn.lock().await.query("SELECT * FROM test_definition").await?.take(0)
    }


    pub async fn delete_by_name(&self, name: String) -> Result<(), DbError> {
        self.conn.lock().await
            .query("DELETE FROM test_definition WHERE name = $name")
            .bind(("name", name))
            .await
            .map_err(DbError::Operation)?;

    
        Ok(())
    }

    pub async fn clean_by_name(&self, test_def: TestDefinition) -> Result<(), DbError> {


        let test_id = test_def.id
            .ok_or(DbError::MissingField("TestDefinition ID"))?;

        let mut delete_list: Vec<Thing> = Vec::new();


        // Get things for the test executions
        let mut result_exec = self.conn.lock().await
            .query("RETURN (
                      SELECT out FROM defines WHERE in = $id
                    ).out;" )
            .bind(("id", test_id.clone()))
            .await
            .map_err(DbError::Operation).unwrap();
        let exec_id_list: Vec<Thing> = result_exec.take(0).unwrap();

        delete_list.extend(exec_id_list.iter().cloned());

        for exec_id in exec_id_list{

            // Get things for the test executions
            let mut result_algo = self.conn.lock().await
                .query("RETURN (
                          SELECT out FROM has_run WHERE in = $id
                        ).out;" )
                .bind(("id", exec_id))
                .await
                .map_err(DbError::Operation).unwrap();

            let algo_run_id_list: Vec<Thing> = result_algo.take(0).unwrap();

            delete_list.extend(algo_run_id_list.iter().cloned());


            for algo_run_id in algo_run_id_list{

                                // Get things for the test executions
                let mut result_iter = self.conn.lock().await
                    .query("RETURN (
                              SELECT out FROM has_iteration WHERE in = $id
                            ).out;" )
                    .bind(("id", algo_run_id))
                    .await
                    .map_err(DbError::Operation).unwrap();
                
                let iter_id_list: Vec<Thing> = result_iter.take(0).unwrap();
                delete_list.extend(iter_id_list.iter().cloned());

                for iter_id in iter_id_list {

                    let iter_id = Arc::new(iter_id);

                    //Get APEs
                    let mut result_ape = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_ape WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut apes: Vec<Thing> = result_ape.take(0).unwrap();

                    //Get RPEs
                    let mut result_rpe = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_rpe WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut rpes: Vec<Thing> = result_rpe.take(0).unwrap();

                    //Get Odometry
                    let mut result_odom = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_odometry WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut odoms: Vec<Thing> = result_odom.take(0).unwrap();          

                    //Get Position
                    let mut result_pos = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_position WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut positions: Vec<Thing> = result_pos.take(0).unwrap();              
                    
                    //Get Metric
                    let mut result_metric = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_metric WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut metrics: Vec<Thing> = result_metric.take(0).unwrap();

                    //Get Stats
                    let mut result_stat = self.conn.lock().await
                        .query("RETURN (
                                  SELECT out FROM has_stat WHERE in = $name
                                ).out;" )
                        .bind(("name", iter_id.clone()))
                        .await
                        .map_err(DbError::Operation).unwrap();
                    let mut stats: Vec<Thing> = result_stat.take(0).unwrap();

                    delete_list.append(&mut apes);
                    delete_list.append(&mut rpes);
                    delete_list.append(&mut stats);
                    delete_list.append(&mut metrics);
                    delete_list.append(&mut odoms);
                    delete_list.append(&mut positions);

                }



            }


        }

        //Delete all items
        self.conn.lock().await
            .query("DELETE $items" )
            .bind(("items", delete_list))
            .await
            .map_err(DbError::Operation).unwrap();

        Ok(())
    
    }

}