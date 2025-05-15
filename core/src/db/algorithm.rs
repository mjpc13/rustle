use std::sync::Arc;

use surrealdb::{engine::local::Db, Surreal};
use tokio::sync::Mutex;

use crate::{models::Algorithm, services::DbError};

pub struct AlgorithmRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}


impl AlgorithmRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, algorithm: &mut Algorithm) -> Result<(), surrealdb::Error> {
        let created: Option<Algorithm> = self.conn.lock().await
            .create("algorithm")
            .content(algorithm.clone())
            .await?;

            if let Some(created) = created {
                algorithm.id = created.id;
            }

        Ok(())
    
    }  

    pub async fn get_by_name(&self, name: String) -> Result<Option<Algorithm>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM algorithm WHERE name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn load(&self, id: &str) -> Result<Option<Algorithm>, surrealdb::Error> {
        self.conn.lock().await.select(("algorithm", id)).await
    }

    pub async fn list_all(&self) -> Result<Vec<Algorithm>, surrealdb::Error> {
        self.conn.lock().await.query("SELECT * FROM algorithm").await?.take(0)
    }

    pub async fn delete_by_name(&self, name: String) -> Result<(), DbError> {
        self.conn.lock().await
            .query("DELETE FROM algorithm WHERE name = $name")
            .bind(("name", name))
            .await
            .map_err(DbError::Operation)?;
    
        Ok(())
    }


}