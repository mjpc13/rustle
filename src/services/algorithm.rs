use std::fs::File;

use crate::{models::Algorithm, db::AlgorithmRepo, services::error::{AlgorithmError, ValidationError}};

use super::error::ProcessingError;

pub struct AlgorithmService {
    repo: AlgorithmRepo,
}

impl AlgorithmService {
    pub fn new(repo: AlgorithmRepo) -> Self {
        Self { repo }
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

    // Add more business logic methods as needed
}