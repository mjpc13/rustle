use std::fs::File;
use crate::{db::DatasetRepo, models::{Dataset, Odometry}};

use super::error::ProcessingError; 

#[derive(Clone)]
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


    pub async fn add_ground_truth(
        &self,
        dataset: &Dataset,
        odom: Odometry,
    ) -> Result<(), ProcessingError> {
        let dataset_id = dataset.id.as_ref()
            .ok_or(ProcessingError::MissingField("Dataset ID".to_owned()))?;
    
        self.repo.append_ground_truth(dataset_id, odom)
            .await
            .map_err(|e| ProcessingError::Database(e))
    }

    pub async fn get_all(&self) -> Result<Vec<Dataset>, ProcessingError> {
        let results = self.repo.list_all().await?;
        Ok(results)
    }

    pub async fn delete_dataset_by_name(&self, name: &String){
        self.repo.delete_by_name(name.to_string()).await;
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