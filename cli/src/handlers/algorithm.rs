use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{
    models::{RosVersion, Algorithm, slam_config::SLAMConfig}, 
    services::{AlgorithmService, params::ParamsService}, 
};

use crate::args::{AlgoCommand, AlgoSubCommand};

use serde_yaml::from_reader;
use std::{error::Error, fs::File};

#[derive(Debug, serde::Deserialize)]
struct AlgorithmConfig {
    algorithms: Vec<Algorithm>,
}

#[derive(Debug, serde::Deserialize)]
struct AlgorithmTemp {
    pub name: String,
    pub image_name: String,
    pub version: String,
    #[serde(default)]
    pub ros_version: RosVersion,
    pub parameters: String,
    pub odom_topics: Vec<String>
}

#[derive(Debug, serde::Deserialize)]
struct AlgorithmConfigTemp {
    algorithms: Vec<AlgorithmTemp>,
}

pub async fn handle_algo(
    cmd: AlgoCommand,
    service: &AlgorithmService,
    param_service: &ParamsService,
) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        AlgoSubCommand::Add(add) => {
            let algo_config = if let Some(file_path) = add.file {
                load_yaml_config(&file_path, param_service).await?
            } else {
                AlgorithmConfig {
                    algorithms: vec![Algorithm {
                        id: None,
                        name: add.name.expect("Missing: --name"),
                        version: add.version.expect("Missing: --version"),
                        image_name: add.image_name.expect("Missing: --image-name"),
                        ros_version: add.ros_version.unwrap_or_else(|| RosVersion::Ros1),
                        //current_params: add.parameters.expect("Missing: --parameters").to_string(),
                        current_params: None,
                        param_list: Vec::new(),
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
                //"Parameters", 
                "Odom Topics"
            ]);

            for algo in algorithms {
                table.add_row(vec![
                    algo.name,
                    algo.version,
                    algo.image_name,
                    //algo.current_params.expect("REASON").to_string(),
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

async fn load_yaml_config(path: &str, service: &ParamsService) -> Result<AlgorithmConfig, Box<dyn Error>> {
    let file = File::open(path).expect("Could not open file");
    //Ok(from_reader(file)?);

    // Reade from file to a YAML object
    // From the config path add a new SLAMConfig to database and return its ID
    // Create a Vec<Algorithm> from the YAML data 

    //let new_config: Option<SLAMConfig> = service.create_from_yaml(path);

    let mut new_configs: AlgorithmConfigTemp = from_reader(file).expect("Could not read config file");

    let mut new_algorithms_config = AlgorithmConfig {algorithms : Vec::new()};

    //println!("read the file");


    for algo in &mut new_configs.algorithms {
        let current_config: SLAMConfig = service.create_from_yaml(&algo.parameters).await?;

        //println!("created the config");

        let mut current_algo = Algorithm {
            id: None,
            name: algo.name.clone(),
            version: algo.version.clone(),
            image_name: algo.image_name.clone(),
            current_params: current_config.id.clone(),
            ros_version: algo.ros_version,
            odom_topics: algo.odom_topics.clone(),
            param_list: Vec::new(),
        };

        new_algorithms_config.algorithms.push(current_algo);
        //println!("Hello there");
    }

    Ok(new_algorithms_config)


}

async fn process_algorithms(service: &AlgorithmService, config: AlgorithmConfig) {
    for mut algorithm in config.algorithms {
        match service.create_algorithm(&mut algorithm).await {
            Ok(_) => info!("Created algorithm: {}", algorithm.name),
            Err(e) => error!("Failed to create algorithm {}: {}", algorithm.name, e),
        }
    }
}

