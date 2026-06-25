use std::fs::File;
use crate::{db::DatasetRepo, models::{Dataset, Odometry}, services::DbError};

use surrealdb::sql::Thing;

use super::error::ProcessingError;
use directories::ProjectDirs;
use log::{info, warn};
use tokio::fs::read_dir;
use yaml_rust2::YamlLoader;
use std::error::Error;
use std::io::Write;
use csv::ReaderBuilder;
use std::path::Path;
use std::fs;

use crate::utils::rosbag_reader;
use crate::models::ros::Tum;
use crate::models::ros::ros_msg::RosMsg;


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

        let mut full_dataset_path = String::new();

        //Create a new dataset folder
        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(ProcessingError::IO("Unable to find application path".to_owned()))?;
            full_dataset_path = format!("{data_dir_str}/data/dataset_{}", dataset.name);

            //Create the directories if they dont exist
            fs::create_dir_all(&full_dataset_path).unwrap();

        };

        let topic = dataset.ground_truth_topic.clone().ok_or(ProcessingError::InvalidDataset("Missing groundtruth topic in dataset definition!".to_owned()))?;

        //Read the dataset groundtruth and write it in the dataset folder!
        let _ = rosbag_reader::read_rosbag_py(&dataset.dataset_path, &format!("{}/groundtruth",&full_dataset_path), &topic, dataset.ros_version).unwrap();

        // Read the TUM files as a CSV
        let file = File::open(&format!("{}/groundtruth",&full_dataset_path)).map_err(|e| ProcessingError::IO(format!("Unable to open Groundtruth positions file: {e}")))?;
        let mut rdr = ReaderBuilder::new()
            .has_headers(false) // set to false if your CSV has no headers
            .delimiter(b' ')
            .from_reader(file);

        let mut odom_list: Vec<Odometry> = vec![];
        // Iterate over each record
        for result in rdr.deserialize() {
            let record: Tum = result.map_err(|e| ProcessingError::MissingField(format!("Unable to convert TUM entry to TUM object: {e}")))?;
            //println!("{:?}", record);
            let odom = RosMsg::Tum(record).as_odometry().map_err(|r| ProcessingError::DefinitionMismatch(format!("{r}")))?;
            odom_list.push(odom);
        }

        dataset.ground_truth = Some(odom_list);


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

    pub async fn get_dataset_id_by_name(&self, name: String) -> Result<Option<Thing>, ProcessingError> {
        //println!("Trying to retrieve all records");
        let current_datasets = self.get_all().await?;

        //println!("{:?}", current_datasets);

        for ds in current_datasets.clone() {
            //let trimmed = &name.clone()[1..name.clone().len()-1];
            //println!("Target: {}, current name: {}", trimmed.to_string(), ds.name.clone().to_string());

            if ds.name.to_string() == name {
                //println!("Found it");
                return Ok(ds.id);
            }
        }

        return Err(ProcessingError::NotFound(String::from("Dataset not found")));
    }

    pub async fn get_dataset_name_by_id(&self, id: &Option<Thing>) -> Result<Option<String>, DbError> {
        match self.repo.get_by_id(id.clone()).await {
            Ok(Some(ds)) => {
                Ok(Some(ds.name))
            },
            Ok(None) => {
                Ok(None)
            },
            Err(e) => {
                Err(e)
            }
        }
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
