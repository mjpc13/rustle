use serde::{Deserialize, Serialize};
use directories::ProjectDirs;
use std::{fs, path::PathBuf};
use toml;
use std::error::Error;

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Config {
    pub database: DatabaseConfig,
    pub docker: DockerConfig,
    pub logging: LoggingConfig,
    pub data: DataConfig,
    pub plotting: PlottingConfig
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct DatabaseConfig {
    pub path: String,  // Store the full path directly
    pub namespace: String,
    pub name: String
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct DockerConfig {
    pub socket: String,
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct LoggingConfig {
    pub level: String,
}


#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct DataConfig {
    pub path: String,  // Store the full path directly
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct PlottingConfig {
    pub width: u32,  // Store the full path directly
    pub height: u32,
}


impl Default for Config {
    fn default() -> Self {
        // Platform-specific directory resolution using `directories` crate
        let proj_dirs = ProjectDirs::from("org", "FRUC", "RUSTLE")
            .expect("Unable to determine project directories");

        // Construct default database path based on platform
        let database_path = proj_dirs.data_dir().join("db"); // platform-specific path
        let database_path_str = database_path.to_str().unwrap_or_default().to_string();

        let result_path = proj_dirs.data_dir().join("data"); // platform-specific path
        let result_path_str = result_path.to_str().unwrap_or_default().to_string();

        // Default values; using platform-specific directory paths
        Config {
            database: DatabaseConfig {
                path: database_path_str,
                namespace: "rustle".to_owned(),
                name: "prod".to_owned(), // Automatically generated path for each platform
            },
            docker: DockerConfig {
                socket: String::from("unix:///var/run/docker.sock"), // Default Docker socket
            },
            logging: LoggingConfig {
                level: String::from("info"), // Default logging level
            },
            data: DataConfig { path: result_path_str },
            plotting: PlottingConfig { width: 1000, height: 1000 },
        }
    }
}

impl Config {
    // Load config from the config.toml in the config directory
    pub fn load() -> Result<Self, Box<dyn Error>> {
        let proj_dirs = ProjectDirs::from("org", "FRUC", "RUSTLE")
            .ok_or("Unable to determine config directory")?;
        
        // Config file path under the config directory
        let config_path = proj_dirs.config_dir().join("config.toml");

        if !config_path.exists() {
            // If config file doesn't exist, initialize it with defaults
            let default_config = Config::default();
            Self::save(&default_config)?;
        }

        let content = fs::read_to_string(config_path)?;
        let config: Config = toml::from_str(&content)?;
        Ok(config)
    }

    // Save config to the config.toml in the config directory
    pub fn save(config: &Config) -> Result<(), Box<dyn Error>> {
        let proj_dirs = ProjectDirs::from("org", "FRUC", "RUSTLE")
            .ok_or("Unable to determine config directory")?;
        
        // Config file path under the config directory
        let config_path = proj_dirs.config_dir().join("config.toml");

        // Ensure the config directory exists
        fs::create_dir_all(proj_dirs.config_dir())?;

        let toml_string = toml::to_string(config)?;
        fs::write(config_path, toml_string)?;

        Ok(())
    }

    // This function gives the path to the config file for easy reference
    pub fn config_path() -> Option<PathBuf> {
        ProjectDirs::from("org", "FRUC", "RUSTLE").map(|pd| pd.config_dir().join("config.toml"))
    }
}