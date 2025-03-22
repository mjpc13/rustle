use surrealdb::{engine::local::Db, Surreal};

use crate::{models::Algorithm, services::DbError};

pub struct AlgorithmRepo {
    conn: Surreal<Db>,
}


impl AlgorithmRepo {
    pub fn new(conn: Surreal<Db>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, algorithm: &mut Algorithm) -> Result<(), surrealdb::Error> {
        let created: Option<Algorithm> = self.conn
            .create("algorithm")
            .content(algorithm.clone())
            .await?;

            if let Some(created) = created {
                algorithm.id = created.id;
            }

        Ok(())
    
    }  

    pub async fn get_by_name(&self, name: String) -> Result<Option<Algorithm>, DbError> {
        self.conn
            .query("SELECT * FROM algorithm WHERE name = $name")
            .bind(("name", name))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn load(&self, id: &str) -> Result<Option<Algorithm>, surrealdb::Error> {
        self.conn.select(("algorithm", id)).await
    }

    pub async fn list_all(&self) -> Result<Vec<Algorithm>, surrealdb::Error> {
        self.conn.query("SELECT * FROM algorithm").await?.take(0)
    }
}