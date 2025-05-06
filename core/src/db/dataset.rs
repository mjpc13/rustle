use std::sync::Arc;

use surrealdb::{engine::local::Db, Surreal, sql::Thing};
use tokio::sync::Mutex;
use crate::models::{Dataset, Odometry};
use crate::services::DbError; 

#[derive(Clone)]
pub struct DatasetRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl DatasetRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, dataset: &mut Dataset) -> Result<(), surrealdb::Error> {

        let created: Option<Dataset> = self.conn
            .lock().await
            .create("dataset")
            .content(dataset.clone())
            .await?;

            if let Some(created) = created {
                dataset.id = created.id;
            }

        Ok(())
    }

    pub async fn get_by_name(&self, name: String) -> Result<Option<Dataset>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM dataset WHERE name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn append_ground_truth(
        &self,
        dataset_id: &Thing,
        odom: Odometry,
    ) -> Result<(), DbError> {
        self.conn
            .lock()
            .await
            .query("
                UPDATE dataset 
                SET ground_truth = array::concat(ground_truth ?? [], $odom) 
                WHERE id = $id
            ")
            .bind(("id", dataset_id.clone()))
            .bind(("odom", vec![odom]))  // Wrap in vec to handle array concat
            .await
            .map_err(|e| DbError::Operation(e))?;

        Ok(())
    }


    pub async fn load(&self, id: &str) -> Result<Option<Dataset>, surrealdb::Error> {
        self.conn.lock().await.select(("dataset", id)).await
    }

    pub async fn list_all(&self) -> Result<Vec<Dataset>, surrealdb::Error> {
        self.conn.lock().await.query("SELECT * FROM dataset").await?.take(0)
    }


    pub async fn delete_by_name(&self, name: String) -> Result<(), DbError> {
        self.conn.lock().await
            .query("DELETE FROM dataset WHERE name = $name")
            .bind(("name", name))
            .await
            .map_err(DbError::Operation)?;
    
        Ok(())
    }
}