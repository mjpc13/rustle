use std::{fmt::format, sync::Arc};

use tokio::sync::Mutex;

use surrealdb::{engine::local::Db, Surreal, sql::Thing};

use crate::{models::slam_config::SLAMConfig, services::DbError, utils::config};

#[derive(Clone)]
pub struct ParamsRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl ParamsRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, params: &mut SLAMConfig) -> Result<(), DbError> {
        let mut config_exists = false;

        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM params")
            .await.unwrap();

        let configs: Vec<SLAMConfig> = existing.take(0).expect("Failed to retrieve stuff from database");

        if(configs.is_empty()) {
            config_exists = false;
        }
        else {
            for current_config in &configs {
                if current_config.params == params.params {
                    config_exists = true;
                }
            }
        }

        if config_exists == false {
            let created: Option<SLAMConfig> = self.conn
                .lock().await
                .create("params")
                .content(params.clone())
                .await.unwrap();

                if let Some(created) = created {
                    params.id = created.id;
                }

            Ok(())
        }
        else {
            if let Some(existing_config) = configs.clone().iter().find(|cfg| *cfg == params) {
                params.id = existing_config.id.clone();
                Ok(())
            }
            else {
                Ok(())
            }
        }
    }

    pub async fn get_by_id(&self, id: Thing) -> Result<SLAMConfig, DbError> {
        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM params")
            .await.unwrap();


        let params: Vec<SLAMConfig> = existing.take(0).expect("An error ocurred while trying to retrieve SLAM configs from the database");

        if let Some(existing_config) = params.clone().iter().find(|cfg| cfg.id.as_ref().unwrap() == &id) {
            Ok(existing_config.clone())
        }
        else {
            Err(DbError::NotFound(String::from("Config not found in the database")))
        }
    }
}