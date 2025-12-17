use std::{fmt::format, sync::Arc};

use tokio::sync::Mutex;

use surrealdb::{engine::local::Db, Surreal, sql::Thing, Error};
use serde_json::{Value, Map};

use crate::{db, models::slam_config::SLAMConfig, services::DbError, utils::config};

use surrealdb::sql::Value as SurrealValue;

use std::collections::HashMap;

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

    pub async fn get_by_id(&self, id: Option<Thing>) -> Result<Option<SLAMConfig>, DbError> {
        self.conn.lock().await
            .query("SELECT * FROM params WHERE id = $id")
            .bind(("id", id))
            .await?
            .take(0)
            .map_err(|e| DbError::Operation(e))
    }

    pub async fn change_param_value(&self, id: Thing, param_key: String, new_value: Value) -> Result<(), DbError> {
        let mut existing = self.conn
            .lock().await
            .query("SELECT * FROM params where id = $id")
            .bind(("id", id.clone()))
            .await.unwrap();

        let params: Vec<SLAMConfig> = existing.take(0).expect("An error ocurred while trying to retrieve SLAM configs from the database");

        let mut found_config = params[0].params.clone();
        //println!("{:?}", found_config);

        if let Some(value) = params[0].params.get(&param_key) {
            //println!("Key found in hashmap: {} -> {}", param_key, value);
            found_config.insert(param_key.clone(), new_value.clone());

            let new_params = serde_json::json!({
                param_key: new_value
            });

            let query = format!("UPDATE {} SET params = $params", id);

            let test_query_exec = self.conn
                                        .lock().await
                                        .query(&query)
                                        .bind(("params", found_config))
                                        .await?;

            //println!("Value updated");

            return Ok(());
        }
        else {
            //panic!("Key not found in hashmap")
            println!("Key {} not found", param_key);
            return Err(DbError::MissingField("Key not found in hashmap"))
        }
        
        Ok(())
    }

    pub async fn change_all_params(&self, params_id: Option<Thing>, new_params: HashMap<String, Value>) -> Result<(), DbError> {
        let test_query_exec = self.conn
                                    .lock().await
                                    .query("UPDATE $params_id SET params = $params")
                                    .bind(("params_id", params_id.clone().unwrap()))
                                    .bind(("params", new_params))
                                    .await?;

        Ok(())
    }
}