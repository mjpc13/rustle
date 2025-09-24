use crate::{args::{TuneCommand, TuneSubCommand}, handlers::tune};

use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{services::tuning::TuningService, models::Dataset};
use chrono::Utc;
use serde_yaml::from_reader;
use std::{fs::File, error::Error};

pub async fn handle_tune(tune_cmd: TuneCommand, service: &TuningService) -> Result<(), Box<dyn Error>> {
    match tune_cmd.command {
        TuneSubCommand::Add(cfg) => {
            println!("Not implemented yet!");
        }

        TuneSubCommand::List => {
            //println!("Not implemented yet!");

            let tuning_configs = service.get_all().await?;
        
            if tuning_configs.is_empty() {
                println!("No datasets found.");
                return Ok(());
            }

            let mut table = Table::new();
            table.load_preset(UTF8_FULL);
            table.set_content_arrangement(ContentArrangement::Dynamic);
            table.set_header(vec![
                "Algo name",
                "Dataset name"
            ]);

            //table.add_row(vec!["Grid Search", "Stuff"]);

            for tune_cfg in tuning_configs {
                table.add_row(vec![
                    tune_cfg.algo_name,
                    tune_cfg.dataset_name,
                ]);
            }

            println!("{table}");
        }
    }

    Ok(())
}