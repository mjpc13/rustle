use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{services::DatasetService, models::Dataset};
use crate::args::{DatasetCommand, DatasetSubCommand, AddDataset, DeleteDataset};
use chrono::Utc;
use serde_yaml::from_reader;
use std::{fs::File, error::Error};


#[derive(Debug, serde::Deserialize)]
struct DatasetConfig {
    datasets: Vec<Dataset>,
}


pub async fn handle_dataset(
    cmd: DatasetCommand,
    service: &DatasetService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        DatasetSubCommand::Add(add) => {
            let dataset_config = if let Some(file_path) = add.file {
                //let f = File::open(file_path)?;
                let datasets: DatasetConfig = load_yaml_config(&file_path)?;
                datasets

            } else {
                DatasetConfig{
                    datasets: vec![
                        Dataset {
                            id: None,
                            name: add.name.expect("Missing: --name"),
                            ground_truth_topic: Some(add.ground_truth_topic.expect("Missing: --ground-truth-topic")),
                            dataset_path: add.dataset_path.expect("Missing: --dataset-path"),
                            ground_truth: None,
                            created_at: Utc::now(),
                        }
                    ]
                }
            };

            process_datasets(&service, dataset_config).await;
        }

        DatasetSubCommand::List => {
            let datasets = service.get_all().await?;
        
            if datasets.is_empty() {
                println!("No datasets found.");
                return Ok(());
            }
        
            let mut table = Table::new();
            table.load_preset(UTF8_FULL);
            table.set_content_arrangement(ContentArrangement::Dynamic);
            table.set_header(vec![
                "Name", 
                "Ground Truth Topic", 
                "Dataset Path", 
                "Created At"
            ]);
        
            for ds in datasets {
                table.add_row(vec![
                    ds.name,
                    ds.ground_truth_topic.clone().unwrap_or_else(|| "N/A".to_string()),
                    ds.dataset_path,
                    ds.created_at.to_rfc3339(),
                ]);
            }
        
            println!("{table}");
        }
        

        DatasetSubCommand::Delete(del) => {
            service.delete_dataset_by_name(&del.name).await;
            println!("Deleted dataset '{}'", del.name);
        }

        DatasetSubCommand::Update(_) => {
            todo!("Update not implemented yet");
        }
    }

    Ok(())
}


fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}

async fn process_datasets(service: &DatasetService, config: DatasetConfig) {
    for mut dataset in config.datasets {
        dataset.created_at = Utc::now();
        match service.create_dataset(&mut dataset).await {
            Ok(_) => info!("Created dataset: {}", dataset.name),
            Err(e) => error!("Failed to create dataset {}: {}", dataset.name, e),
        }
    }
}