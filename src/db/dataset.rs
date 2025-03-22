use surrealdb::{engine::local::Db, Surreal};
use crate::models::Dataset;
use crate::services::DbError; 

pub struct DatasetRepo {
    conn: Surreal<Db>,
}

impl DatasetRepo {
    pub fn new(conn: Surreal<Db>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, dataset: &mut Dataset) -> Result<(), surrealdb::Error> {

        let created: Option<Dataset> = self.conn
            .create("dataset")
            .content(dataset.clone())
            .await?;

            if let Some(created) = created {
                dataset.id = created.id;
            }

        Ok(())
    }

    pub async fn get_by_name(&self, name: String) -> Result<Option<Dataset>, DbError> {
        self.conn
            .query("SELECT * FROM dataset WHERE name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }


    pub async fn load(&self, id: &str) -> Result<Option<Dataset>, surrealdb::Error> {
        self.conn.select(("dataset", id)).await
    }

    pub async fn list_all(&self) -> Result<Vec<Dataset>, surrealdb::Error> {
        self.conn.query("SELECT * FROM dataset").await?.take(0)
    }
}