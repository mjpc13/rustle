use std::collections::HashMap;
use std::{fs::File, sync::Arc};

use bollard::{image::CreateImageOptions, Docker};
use bollard::errors::Error as DockerError;
use log::{info, trace, warn};

use crate::models::slam_config::SLAMConfig;
use crate::{models::Algorithm, db::params::ParamsRepo, services::error::{ValidationError}};
use futures_util::stream::{StreamExt};
use super::error::ProcessingError;

use crate::services::DbError;
use surrealdb::sql::Thing;
use serde_json::Value;

#[derive(Clone)]
pub struct ParamsService {
    pub repo: ParamsRepo,
    docker: Arc<Docker>
}

impl ParamsService {
    pub fn new(repo: ParamsRepo, docker: Arc<Docker>) -> Self {
        Self { repo, docker }
    }

    pub async fn create_from_yaml(&self, yaml_path: &str) -> Result<SLAMConfig, Box<dyn std::error::Error>> {
        let file = File::open(yaml_path)?;
        let mut params_config: SLAMConfig = SLAMConfig::new(yaml_path).unwrap();

        self.repo.save(&mut params_config).await?;

        Ok(params_config)
    }

    pub async fn get_by_id(&self, id: Option<Thing>) -> Result<Option<SLAMConfig>, DbError> {
        self.repo.get_by_id(id).await
    }

    /*
    pub async fn duplicate(&self, id: Option<Thing>) -> Result<Option<Thing>, DbError> {
        if let Some(mut slam_params) = self.repo.get_by_id(id).await? {
            slam_params.id = None;
            self.repo.save(&mut slam_params).await?;
            Ok(slam_params.id)
        }
        else {
            return Err(DbError::NotFound(String::from("SLAM config not found in database")));
        }
    }
    */

    pub async fn update_params(&self, id: Option<Thing>, new_params: &HashMap<String, Value>) -> Result<(), DbError> {
        self.repo.change_all_params(id, new_params.clone()).await?;
        Ok(())
    }
}