use std::{fs::File, sync::Arc};

use bollard::{image::CreateImageOptions, Docker};
use bollard::errors::Error as DockerError;
use log::{info, trace, warn};

use crate::db::TuningRepo;
use crate::models::slam_config::SLAMConfig;
use crate::models::tuning_config::{self, TuningConfig};
use crate::services::DbError;
use crate::{models::Algorithm, db::params::ParamsRepo, services::error::{ValidationError}};
use futures_util::stream::{StreamExt};
use super::error::ProcessingError;

#[derive(Clone)]
pub struct TuningService {
    pub repo: TuningRepo,
    docker: Arc<Docker>
}

impl TuningService {
    pub fn new(repo: TuningRepo, docker: Arc<Docker>) -> Self {
        Self { repo, docker }
    }

    pub async fn create_from_yaml(&self, file_name: &str) -> Result<TuningConfig, Box<dyn std::error::Error>> {
        let mut tuning_config = TuningConfig::new();

        self.repo.save(&mut tuning_config).await?;

        Ok(tuning_config)
    }

    pub async fn save_to_db(&self, config: &mut TuningConfig) -> Result<(), DbError> {
        self.repo.save(config).await?;

        Ok(())
    }

    pub async fn get_all(&self) -> Result<Vec<TuningConfig>, DbError> {
        let results = self.repo.get_all().await?;
        Ok(results)
    }
}