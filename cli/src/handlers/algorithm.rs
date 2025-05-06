use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{models::Algorithm, services::AlgorithmService};
use crate::args::{AddAlgorithm, AlgoCommand, AlgoSubCommand};
use chrono::Utc;
use serde_yaml::from_reader;
use std::{error::Error, fs::File};

#[derive(Debug, serde::Deserialize)]
struct AlgorithmConfig {
    algorithms: Vec<Algorithm>,
}

pub async fn handle_algo(
    cmd: AlgoCommand,
    service: &AlgorithmService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        AlgoSubCommand::Add(add) => {
            let algo_config = if let Some(file_path) = add.file {
                load_yaml_config(&file_path)?
            } else {
                AlgorithmConfig {
                    algorithms: vec![Algorithm {
                        id: None,
                        name: add.name.expect("Missing: --name"),
                        version: add.version.expect("Missing: --version"),
                        image_name: add.image_name.expect("Missing: --image-name"),
                        parameters: add.parameters.expect("Missing: --parameters"),
                        odom_topics: add.odom_topics,
                    }],
                }
            };

            process_algorithms(service, algo_config).await;
        }

        AlgoSubCommand::List => {
            let algorithms = service.get_all().await?;

            if algorithms.is_empty() {
                println!("No algorithms found.");
                return Ok(());
            }

            let mut table = Table::new();
            table.load_preset(UTF8_FULL);
            table.set_content_arrangement(ContentArrangement::Dynamic);
            table.set_header(vec![
                "Name", 
                "Version", 
                "Image", 
                "Parameters", 
                "Odom Topics"
            ]);

            for algo in algorithms {
                table.add_row(vec![
                    algo.name,
                    algo.version,
                    algo.image_name,
                    algo.parameters,
                    format!("{:?}", algo.odom_topics),
                ]);
            }

            println!("{table}");
        }

        AlgoSubCommand::Delete(del) => {
            service.delete_algo_by_name(&del.name).await;
            println!("Deleted algorithm '{}'", del.name);
        }
    }

    Ok(())
}

fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}

async fn process_algorithms(service: &AlgorithmService, config: AlgorithmConfig) {
    for mut algorithm in config.algorithms {
        match service.create_algorithm(&mut algorithm).await {
            Ok(_) => info!("Created algorithm: {}", algorithm.name),
            Err(e) => error!("Failed to create algorithm {}: {}", algorithm.name, e),
        }
    }
}