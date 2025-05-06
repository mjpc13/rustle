use crate::args::{ConfigCommand, ConfigSubCommand, SetConfig};
use rustle_core::config::{Config, DatabaseConfig, DockerConfig, LoggingConfig, ResultsConfig};
use comfy_table::{Table, Cell, presets::UTF8_FULL};
use std::{error::Error, fs};
use toml;

pub async fn handle_config(cmd: ConfigCommand) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        ConfigSubCommand::Show => handle_config_show().await?,
        ConfigSubCommand::Set(set_config) => handle_config_set(set_config).await?,
    }

    Ok(())
}

async fn handle_config_show() -> Result<(), Box<dyn Error>> {

    let config = Config::load()?;

    let mut table = Table::new();
    table.load_preset(UTF8_FULL);
    table.set_header(vec!["Section", "Key", "Value"]);

    // Database section
    table.add_row(vec!["database", "path", &config.database.path]);
    table.add_row(vec!["database", "namespace", &config.database.namespace]);
    table.add_row(vec!["database", "name", &config.database.name]);

    // Docker section
    table.add_row(vec!["docker", "socket", &config.docker.socket]);

    // Logging section
    table.add_row(vec!["logging", "level", &config.logging.level]);

    // Results section
    table.add_row(vec!["results", "path", &config.results.path]);

    println!("{table}");

    Ok(())
}


// Set the configuration value
async fn handle_config_set(set_config: SetConfig) -> Result<(), Box<dyn Error>> {
    // Load the current configuration
    let mut config = Config::load()?;

    // Update the config based on section and key
    match set_config.section.as_str() {
        "database" => update_database_config(&mut config.database, &set_config),
        "docker" => update_docker_config(&mut config.docker, &set_config),
        "logging" => update_logging_config(&mut config.logging, &set_config),
        "results" => update_results_config(&mut config.results, &set_config),
        _ => return Err("Unknown section".into()),
    }

    // Write the updated config back to the TOML file
    save_config(&config)?;

    println!("Configuration updated successfully!");

    Ok(())
}

fn update_database_config(database: &mut DatabaseConfig, set_config: &SetConfig) {
    match set_config.key.as_str() {
        "path" => database.path = set_config.value.clone(),
        "namespace" => database.namespace = set_config.value.clone(),
        "name" => database.name = set_config.value.clone(),
        _ => println!("Invalid key for database configuration"),
    }
}

fn update_docker_config(docker: &mut DockerConfig, set_config: &SetConfig) {
    if set_config.key == "socket" {
        docker.socket = set_config.value.clone();
    } else {
        println!("Invalid key for docker configuration");
    }
}

fn update_logging_config(logging: &mut LoggingConfig, set_config: &SetConfig) {
    if set_config.key == "level" {
        logging.level = set_config.value.clone();
    } else {
        println!("Invalid key for logging configuration");
    }
}

fn update_results_config(results: &mut ResultsConfig, set_config: &SetConfig) {
    if set_config.key == "path" {
        results.path = set_config.value.clone();
    } else {
        println!("Invalid key for results configuration");
    }
}

fn save_config(config: &Config) -> Result<(), Box<dyn Error>> {
    // Serialize the config back to TOML format
    let toml_str = toml::to_string(config)?;

    // Get the path to the config file
    let proj_dirs = directories::ProjectDirs::from("org", "RUSTLE", "rustle")
        .expect("Failed to determine project directories");

    let config_file_path = proj_dirs.config_dir().join("config.toml");

    // Write the updated config to the file
    fs::write(config_file_path, toml_str)?;

    Ok(())
}