use crate::{args::{TuneCommand, TuneSubCommand}, handlers::tune};

use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{services::{tuning::TuningService, AlgorithmService, DatasetService, params::ParamsService}, models::Dataset, models::tuning_config::TuningConfig};
use rustle_core::models::tuning::grid_search::GridSearchConfig;
use chrono::Utc;
use serde_yaml::from_reader;
use std::{error::Error, fs::File, vec};

use serde_json::{Value, Number};
use std::collections::HashMap;
use std::io::BufReader;

use rustle_core::models::parameter_space::ParameterSpaceI32;
use rustle_core::services::error::*;


pub async fn handle_tune(tune_cmd: TuneCommand, service: &TuningService, algo_service: &AlgorithmService, dataset_service: &DatasetService, params_service: &ParamsService) -> Result<(), Box<dyn Error>> {
    match tune_cmd.command {
        TuneSubCommand::Add(cfg) => {

            let mut cfg = load_tune_config(cfg.file.unwrap().as_str(), algo_service, dataset_service, params_service).await;

            match cfg {
                Ok(Some(mut result_cfg)) => {
                    service.save_to_db(&mut result_cfg).await?;

                    let algo_params_id = algo_service.get_by_id(result_cfg.algo_id.clone()).await?.unwrap();
                    let mut initial_config: HashMap<String, Value> = params_service.get_by_id(algo_params_id.current_params).await?.unwrap().params;

                    let mut grid = GridSearchConfig::new();
                    grid.build_configurations(&result_cfg.parameters, &initial_config);

                    info!("Created tuning config");
                }
                Ok(None) => info!("Failed to create tuning config"),
                Err(e) => info!("Failed to create tuning config: {}", e),
            }

        }

        TuneSubCommand::List => {

            let tuning_configs = service.get_all().await?;

            //println!("{:?}", tuning_configs);
        
            if tuning_configs.is_empty() {
                println!("No tuning configurations found.");
                return Ok(());
            }

            let mut table = Table::new();
            table.load_preset(UTF8_FULL);
            table.set_content_arrangement(ContentArrangement::Dynamic);
            table.set_header(vec![
                "Algorithm name",
                "Dataset name"
            ]);

            for tune_cfg in tuning_configs {
                let algo_name: String = algo_service.get_algo_name_by_id(tune_cfg.algo_id).await.unwrap().unwrap();
                let dataset_name: String = dataset_service.get_dataset_name_by_id(tune_cfg.dataset_id).await.unwrap().unwrap();

                table.add_row(vec![
                    algo_name,
                    dataset_name
                ]);
            }

            println!("{table}");
        }
    }

    Ok(())
}

async fn load_tune_config(file_name: &str, algo_service: &AlgorithmService, dataset_service: &DatasetService, params_service: &ParamsService) -> Result<Option<TuningConfig>, Box<dyn Error>> {
    let file = File::open(file_name)?;
    let reader = BufReader::new(file);

    let cfg: HashMap<String, Value> = serde_yaml::from_reader(reader)?;

    let mut new_tuning_config = TuningConfig::new();

    // testing some stuff...
    if let Some(params_object) = cfg.get("parameters") {
        if let Some(params_map) = params_object.as_object() {
            new_tuning_config.parameters = params_map.clone().into_iter().collect();
        }
        else {
            return Err(Box::new(TuningError::NoParametersObject()));
        }
    }
    else {
        return Err(Box::new(TuningError::NoParametersField()));
    }

    if let Some(algo_name) = cfg.get("algo_name") {
        let trimmed_algo_name = algo_name.to_string()[1..algo_name.to_string().len()-1].to_string();
        new_tuning_config.algo_id = algo_service.get_algo_id_by_name(trimmed_algo_name).await?;
    }
    else {
        return Err(Box::new(TuningError::NoAlgoName()));
    }

    if let Some(dataset_name) = cfg.get("dataset_name") {
        let trimmed_dataset_name = dataset_name.to_string()[1..dataset_name.to_string().len()-1].to_string();
        new_tuning_config.dataset_id = dataset_service.get_dataset_id_by_name(trimmed_dataset_name).await?;
    }
    else {
        return Err(Box::new(TuningError::NoDatasetName()));
    }

    let algo_params_config = algo_service.get_by_id(new_tuning_config.algo_id.clone()).await?.unwrap();
    let algo_params: HashMap<String, Value> = params_service.get_by_id(algo_params_config.current_params).await?.unwrap().params;

    new_tuning_config.validate_parameter_space(&algo_params)?;

    Ok(Some(new_tuning_config))
}
