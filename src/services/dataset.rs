use std::fs::File;
use crate::{db::DatasetRepo, models::Dataset};

use super::error::ProcessingError; 

pub struct DatasetService{
    repo: DatasetRepo,
}

impl DatasetService {

    pub fn new(repo: DatasetRepo) -> Self {
        Self { repo }
    }

    /// File system operation moved to service layer
    pub fn load_from_yaml(path: &str) -> Result<Dataset, Box<dyn std::error::Error>> {
        let file = File::open(path)?;
        let mut dataset: Dataset = serde_yaml::from_reader(file)?;
        dataset.id = None;
        Ok(dataset)
    }

    pub async fn create_dataset(&self, dataset: &mut Dataset) -> Result<(), ProcessingError> {
        // Check for existing algorithm
        if let Some(existing) = self.repo.get_by_name(dataset.name.clone()).await? {
            return Err(ProcessingError::Conflict(format!(
                "Dataset '{}' already exists!",
                existing.name
            )));
        }

        // Save new algorithm
        self.repo.save(dataset)
            .await?;
        
        Ok(())
    }

    // Add validation logic
    //pub fn validate_ground_truth(dataset: &Dataset) -> Result<(), ValidationError> {
    //    if let Some(truth) = &dataset.ground_truth {
    //        if truth.is_empty() {
    //            return Err(ValidationError::new("Ground truth cannot be empty"));
    //        }
    //    }
    //    Ok(())
    //}
}