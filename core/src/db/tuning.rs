use std::{fmt::format, sync::Arc};

use tokio::sync::Mutex;

use surrealdb::{engine::local::Db, Surreal, sql::Thing, Error};
use serde_json::{Value, Map};

use crate::{db, models::{slam_config::SLAMConfig, tuning_config::TuningConfig}, services::DbError, utils::config};

use surrealdb::sql::Value as SurrealValue;

use std::collections::HashMap;

use crate::models::metrics::Metric;
use crate::models::tuning::TuneType;

#[derive(Clone)]
pub struct TuningRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl TuningRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, tuning_config: &mut TuningConfig) -> Result<(), DbError> {

        let created: Option<TuningConfig> = self.conn.lock().await
            .create("tuning")
            .content(tuning_config.clone())
            .await?;

        if let Some(created) = created {
            tuning_config.id = created.id;
        }

        Ok(())
    }

    pub async fn get_by_name(&self, name: String) -> Result<TuningConfig, DbError> {
        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM tuning where name = $name")
            .bind(("name", name.clone()))
            .await.unwrap();

        let configs: Vec<TuningConfig> = existing.take(0).expect("An error ocurred while trying to retrieve tuning configs from the database");

        if configs.len() > 0 {
            let mut read_config = configs[0].clone();

            match configs[0].clone().tuning_algo.unwrap().as_str() {
                "grid_search" => {
                    read_config.tuning_type = Some(TuneType::GridSearch(configs[0].tuning_type.clone().unwrap().as_grid_search().unwrap().clone()));
                }
                "random_search" => {
                    read_config.tuning_type = Some(TuneType::RandomSearch(configs[0].tuning_type.clone().unwrap().as_random_search().unwrap().clone()));
                }
                "simulated_annealing" => {
                    read_config.tuning_type = Some(TuneType::SimulatedAnnealing(configs[0].tuning_type.clone().unwrap().as_simulated_annealing().unwrap().clone()));
                }
                _ => {}
            }

            Ok(read_config.clone())
        }
        else {
            Err(DbError::NotFound(String::from("There is no TuningConfig record with that id")))
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

    pub async fn store_results(&self, tuning_id: Option<Thing>, results: Vec<(Vec<Metric>, Vec<(String, u64)>)>) -> Result<(), DbError> {
        let test_query_exec = self.conn
                                    .lock().await
                                    .query("UPDATE $tuning_id SET results = $results")
                                    .bind(("tuning_id", tuning_id.clone().unwrap()))
                                    .bind(("results", results))
                                    .await?;

        Ok(())
    }

    pub async fn get_results(&self, tuning_id: Option<Thing>) -> Result<Vec<(Vec<Metric>, usize)>, DbError> {
        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM tuning where id = $tuning_id")
            .bind(("tuning_id", tuning_id.clone()))
            .await.unwrap();

        let results: Vec<Vec<(Vec<Metric>, usize)>> = existing.take(0).expect("An error ocurred while trying to retrieve tuning results from the database");

        if results.len() > 0 {
            Ok(results[0].clone())
        }
        else {
            Err(DbError::NotFound(String::from("There is no TuningConfig record with that id")))
        }
    }
}