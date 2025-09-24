use std::{fs::File, sync::Arc};

use bollard::{image::CreateImageOptions, Docker};
use bollard::errors::Error as DockerError;
use log::{info, trace, warn};

use crate::db::TuningRepo;
use crate::models::slam_config::SLAMConfig;
use crate::models::tuning_config::TuningConfig;
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

    pub async fn get_all(&self) -> Result<Vec<TuningConfig>, DbError> {
        let results = self.repo.get_all().await?;
        Ok(results)
    }
}