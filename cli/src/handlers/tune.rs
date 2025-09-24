use crate::{args::{TuneCommand, TuneSubCommand}, handlers::tune};

use comfy_table::{presets::UTF8_FULL, ContentArrangement, Table};
use log::{error, info};
use rustle_core::{services::DatasetService, models::Dataset};
use chrono::Utc;
use serde_yaml::from_reader;
use std::{fs::File, error::Error};

pub async fn handle_tune(tune_cmd: TuneCommand) -> Result<(), Box<dyn Error>> {
    match tune_cmd.command {
        TuneSubCommand::Add(cfg) => {
            println!("Not implemented yet!");
        }

        TuneSubCommand::List => {
            println!("Not implemented yet!");
        }
    }

    Ok(())
}