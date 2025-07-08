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
        let _ = self.repo.delete_by_name(name.to_string()).await;
    }


    pub async fn set_duration(&self, dataset: &Dataset) -> Result<(), ProcessingError>{

        //get the ground truth
        let dataset_id = dataset.id.as_ref()
            .ok_or(ProcessingError::MissingField("Dataset ID".to_owned()))?;

        let init_time = dataset.ground_truth.clone()
            .ok_or(ProcessingError::MissingField("Groundtruth Positions".to_owned()))?
            .iter()
            .next()
            .ok_or(ProcessingError::MissingField("Groundtruth Positions".to_owned()))?
            .header.time;

        let end_time = dataset.ground_truth.clone()
            .ok_or(ProcessingError::MissingField("Groundtruth Positions".to_owned()))?
            .iter()
            .last()
            .ok_or(ProcessingError::MissingField("Groundtruth Positions".to_owned()))?
            .header.time;

        let duration = (end_time - init_time).num_milliseconds() as f32 / 1000.0;
    
        let _ = self.repo.set_duration(dataset_id, Some(duration))
            .await
            .map_err(|e| ProcessingError::Database(e));

        Ok(())
    }

}