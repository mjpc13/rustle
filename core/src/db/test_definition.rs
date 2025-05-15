use std::sync::{Arc};

use log::info;
use surrealdb::{engine::local::Db, Surreal};
use tokio::sync::Mutex;
use crate::{models::{TestDefinition, TestExecution}, services::DbError};

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


}