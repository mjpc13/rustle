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
}