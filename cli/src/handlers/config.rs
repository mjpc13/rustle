use crate::args::{ConfigCommand, ConfigSubCommand, SetConfig};
use rustle_core::utils::config::{Config, DatabaseConfig, DockerConfig, LoggingConfig, DataConfig};
use comfy_table::{presets::{UTF8_FULL, UTF8_HORIZONTAL_ONLY}, Cell, Table};
use std::{error::Error, fs};
use toml;
use owo_colors::OwoColorize;


pub async fn handle_config(cmd: ConfigCommand) -> Result<(), Box<dyn Error>> {
    match cmd.command {
        ConfigSubCommand::Show => handle_config_show()?,
        ConfigSubCommand::Set(set_config) => handle_config_set(set_config).await?,
    }

    Ok(())
}

fn format_bool(value: bool) -> String {
    if value {
        "✓".green().to_string()
    } else {
        "✗".red().to_string()
    }
}

fn print_section(header: String, rows: &[(String, String)]) {
    println!("{}", format!("{}", header).bold().underline());


    let mut table = Table::new();
    table.load_preset(UTF8_HORIZONTAL_ONLY);
    table.set_header(vec!["KEY".bold().to_string(), "VALUE".bold().to_string()]);
    for (key, value) in rows {
        table.add_row(vec![key.to_string(), value.clone()]);
    }
    println!("{}\n", table);
}

fn handle_config_show() -> Result<(), Box<dyn std::error::Error>> {
    let config = Config::load()?;

    println!("\n{}\n", "--- Current Configuration ---".bold());

    print_section("DATABASE".blue().to_string(), &[
        ("path".blue().to_string(), config.database.path.clone()),
        ("namespace".blue().to_string(), config.database.namespace.clone()),
        ("name".blue().to_string(), config.database.name.clone()),
    ]);

    print_section("DOCKER".cyan().to_string(), &[
        ("socket".cyan().to_string(), config.docker.socket.clone()),
    ]);

    print_section("LOGGING".yellow().to_string(), &[
        ("level".yellow().to_string(), config.logging.level.clone()),
    ]);

    print_section("DATA".magenta().to_string(), &[
        ("path".magenta().to_string(), config.data.path.clone()),
    ]);

    print_section("PLOTTING".green().to_string(), &[
        ("width".green().to_string(), config.plotting.width.to_string()),
        ("height".green().to_string(), config.plotting.height.to_string()),
        ("show_cut_band".green().to_string(), format_bool(config.plotting.show_cut_band)),
        ("show_drop_band".green().to_string(), format_bool(config.plotting.show_drop_band)),
        ("show_markers".green().to_string(), format_bool(config.plotting.show_markers)),
        ("show_confidence_band".green().to_string(), format_bool(config.plotting.show_confidence_band)),
        ("confidence_color_offset".green().to_string(), config.plotting.confidence_color_offset.to_string()),
    ]);

    print_section("EVO".purple().to_string(), &[
        ("align".purple().to_string(), format_bool(config.evo.align)),
        ("align_origin".purple().to_string(), format_bool(config.evo.align_origin)),
        ("t_max_diff".purple().to_string(), config.evo.t_max_diff.to_string()),
        ("t_offset".purple().to_string(), config.evo.t_offset.to_string()),
        ("scale".purple().to_string(), format_bool(config.evo.scale)),
        ("n_to_align".purple().to_string(), config.evo.n_to_align.to_string()),
    ]);

    print_section("RUSTLE".bright_blue().to_string(), &[
        ("start_offset".bright_blue().to_string(), config.rustle.start_offset.to_string()),
        ("dataset_duration".bright_blue().to_string(), config.rustle.dataset_duration.to_string()),
        ("dataset_start".bright_blue().to_string(), config.rustle.dataset_start.to_string()),
    ]);

    println!("{}\n", "--- End of Configuration ---".bold());

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
        "results" => update_results_config(&mut config.data, &set_config),
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

fn update_results_config(data: &mut DataConfig, set_config: &SetConfig) {
    if set_config.key == "path" {
        data.path = set_config.value.clone();
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