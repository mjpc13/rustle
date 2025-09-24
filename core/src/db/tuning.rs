use std::{fmt::format, sync::Arc};

use tokio::sync::Mutex;

use surrealdb::{engine::local::Db, Surreal, sql::Thing, Error};
use serde_json::{Value, Map};

use crate::{db, models::{slam_config::SLAMConfig, tuning_config::TuningConfig}, services::DbError, utils::config};

use surrealdb::sql::Value as SurrealValue;

use std::collections::HashMap;

#[derive(Clone)]
pub struct TuningRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl TuningRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, tuning_config: &mut TuningConfig) -> Result<(), DbError> {
        let mut config_exists = false;

        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM tuning")
            .await.unwrap();

        let configs: Vec<TuningConfig> = existing.take(0).expect("Failed to retrieve stuff from database");

        if configs.is_empty() {
            config_exists = false;
        }
        else {
            for current_config in &configs {
                if *current_config == *tuning_config {
                    config_exists = true;
                }
            }
        }

        if config_exists == false {
            let created: Option<SLAMConfig> = self.conn
                .lock().await
                .create("tuning")
                .content(tuning_config.clone())
                .await.unwrap();

                if let Some(created) = created {
                    tuning_config.id = created.id;
                }

            Ok(())
        }
        else {
            return Err(DbError::NotFound(String::from("Config already exists in the database")));
        }
    }

    pub async fn get_by_id(&self, id: Thing) -> Result<TuningConfig, DbError> {
        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM tuning where id = $id")
            .bind(("id", id.clone()))
            .await.unwrap();

        let configs: Vec<TuningConfig> = existing.take(0).expect("An error ocurred while trying to retrieve SLAM configs from the database");

        if configs.len() > 0 {
            Ok(configs[0].clone())
        }
        else {
            Err(DbError::NotFound(String::from("There is no TuningConfig record with that id")))
        }
    }

    pub async fn get_all(&self) -> Result<Vec<TuningConfig>, surrealdb::Error> {
        self.conn.lock().await.query("SELECT * FROM tuning").await?.take(0)
    }
}