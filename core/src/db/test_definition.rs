use std::sync::{Arc};

use surrealdb::{engine::local::Db, Surreal};
use tokio::sync::Mutex;
use crate::models::TestDefinition;

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




}