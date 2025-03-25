use std::{fs::File, sync::Arc};

use bollard::{image::CreateImageOptions, Docker};
use bollard::errors::Error as DockerError;
use log::{info, trace, warn};
use tokio::sync::Mutex;

use crate::{models::Algorithm, db::AlgorithmRepo, services::error::{AlgorithmError, ValidationError}};
use futures_util::stream::{StreamExt};
use futures_util::future;
use super::error::ProcessingError;

pub struct AlgorithmService {
    repo: AlgorithmRepo,
    docker: Arc<Docker>
}

impl AlgorithmService {
    pub fn new(repo: AlgorithmRepo, docker: Arc<Docker>) -> Self {
        Self { repo, docker }
    }

    /// Load algorithm configuration from YAML file and save to DB
    pub async fn create_from_yaml(
        &self,
        yaml_path: &str
    ) -> Result<Algorithm, Box<dyn std::error::Error>> {
        // Read and parse YAML
        let file = File::open(yaml_path)?;
        let mut algorithm: Algorithm = serde_yaml::from_reader(file)?;
        
        
        // Validate
        self.validate(&algorithm)?;
        
        // Persist
        self.repo.save(&mut algorithm).await?; //TODO: Save in database
        Ok(algorithm)
    }

    pub async fn create_algorithm(&self, algorithm: &mut Algorithm) -> Result<(), ProcessingError> {
        // Check for existing algorithm
        if let Some(existing) = self.repo.get_by_name(algorithm.name.clone()).await? {
            return Err(ProcessingError::Conflict(format!(
                "Algorithm '{}' already exists!",
                existing.name
            )));
        }


        // Verify Docker image exists
        match self.verify_docker_image(&algorithm.image_name).await {
            Ok(_) => info!("Docker image verified"),
            Err(ProcessingError::ImageNotFound(_)) => {
                self.pull_image(&algorithm.image_name).await?;
            }
            Err(e) => return Err(e),
        }



        // Save new algorithm
        self.repo.save(algorithm)
            .await?;
        
        Ok(())
    }

    fn validate(&self, algorithm: &Algorithm) -> Result<(), ValidationError> {
        if algorithm.name.is_empty() {
            return Err(ValidationError("Algorithm name cannot be empty".into()));
        }
        if algorithm.version.is_empty() {
            return Err(ValidationError("Version cannot be empty".into()));
        }
        Ok(())
    }
    
    async fn verify_docker_image(&self, image: &str) -> Result<(), ProcessingError> {
        
        match self.docker.inspect_image(image).await {
            Ok(_) => Ok(()),
            Err(DockerError::DockerResponseServerError { status_code, .. }) if status_code == 404 => {
                Err(ProcessingError::ImageNotFound(image.to_string()))
            }
            Err(e) => Err(ProcessingError::DockerOperation(format!(
                "Verification failed: {}",
                e
            ))),
        }
    }


    //Pulls the docker image
    async fn pull_image(&self, img_name: &str) -> Result<(), ProcessingError> {
        let options = Some(CreateImageOptions {
            from_image: img_name,
            ..Default::default()
        });
    
        info!("Pulling Docker image '{}' from registry", img_name);
        
        let mut pull_stream = self.docker.create_image(options, None, None);
        let mut last_progress = None;
    
        while let Some(result) = pull_stream.next().await {
            match result {
                Ok(info) => {

                    match info.progress{
                        Some(p) => trace!("Docker pull progress: {p}"),
                        None => (),
                    }

                    if let Some(progress) = info.progress_detail.as_ref() {
                        last_progress = progress.current.map(|c| (c, progress.total.unwrap_or(0)));
                    }
                }
                Err(e) => {
                    warn!("Failed to pull Docker image '{}': {}", img_name, e);
                    return Err(ProcessingError::DockerOperation(format!(
                        "Image pull failed: {}",
                        e
                    )));
                }
            }
        }
    
        if let Some((current, total)) = last_progress {
            if current < total {
                warn!("Incomplete Docker image pull for '{}'", img_name);
                return Err(ProcessingError::DockerOperation(
                    "Partial image download detected".into()
                ));
            }
        }
    
        info!("Successfully pulled Docker image '{}'", img_name);
        Ok(())
    }


}