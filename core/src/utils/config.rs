use charming::element::Symbol;
use serde::{Deserialize, Serialize};
use directories::ProjectDirs;
use std::{fmt, fs, path::PathBuf, str::FromStr};
use toml;
use std::error::Error;

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Config {
    pub database: DatabaseConfig,
    pub docker: DockerConfig,
    pub logging: LoggingConfig,
    pub data: DataConfig,
    pub plotting: PlottingConfig,
    pub evo: EvoConfig,
    pub rustle: RustleConfig
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
    pub smooth: bool,
    pub y_axis_label_size: u8,
    pub x_axis_label_size: u8,
    pub y_axis_title_size: u8,
    pub x_axis_title_size: u8,
    pub show_legend: bool,
    pub legend_size: u8,
    pub show_band: bool,
    pub marker_type: MarkerType,
    pub marker_size: f32,
    pub show_confidence_band: bool,
    pub confidence_color_offset: u8,
}


#[derive(Debug, Deserialize, Serialize, Clone, Copy)]
pub enum MarkerType{
    Circle,
    Rect,
    RoundRect,
    Triangle,
    Diamond,
    Pin,
    Arrow,
    None
}

impl FromStr for MarkerType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "circle" => Ok(MarkerType::Circle),
            "rect" => Ok(MarkerType::Rect),
            "roundrect" => Ok(MarkerType::RoundRect),
            "triangle" => Ok(MarkerType::Triangle),
            "diamond" => Ok(MarkerType::Diamond),
            "none" => Ok(MarkerType::None),
            "pin" => Ok(MarkerType::Pin),
            "arrow" => Ok(MarkerType::Arrow),
            _ => Err(format!("Invalid MarkerType: {}", s)),
        }
    }
}

impl From<MarkerType> for Symbol {
    fn from(marker: MarkerType) -> Self {
        match marker {
            MarkerType::Circle => Symbol::Circle,
            MarkerType::Rect => Symbol::Rect,
            MarkerType::RoundRect => Symbol::RoundRect,
            MarkerType::Triangle => Symbol::Triangle,
            MarkerType::Diamond => Symbol::Diamond,
            MarkerType::None => Symbol::None,
            MarkerType::Pin => Symbol::Pin,
            MarkerType::Arrow => Symbol::Arrow,
        }
    }
}

impl fmt::Display for MarkerType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            MarkerType::Circle => "circle",
            MarkerType::Rect => "rect",
            MarkerType::RoundRect => "roundrect",
            MarkerType::Triangle => "triangle",
            MarkerType::Diamond => "diamond",
            MarkerType::Pin => "pin",
            MarkerType::Arrow => "arrow",
            MarkerType::None => "none",
        };
        write!(f, "{}", s)
    }
}






#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct EvoConfig {
    pub align: bool,  // Store the full path directly
    pub align_origin: bool,
    pub t_max_diff: f64,
    pub t_offset: f64,
    pub scale: bool,
    pub n_to_align: u32
}

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct RustleConfig {
    pub start_offset: f32, //The offset to start playing the bag
    pub dataset_duration: f32, // Duration to play the dataset, default is maximum duration (None)
    pub dataset_start: f32, // Start the dataset from N seconds
    pub time_precision: f32, // Time precision to have the results matched in seconds, default 0.01s
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
            plotting: PlottingConfig { 
                width: 1000, 
                height: 1000, 
                show_band: true, 
                marker_type: MarkerType::Circle, 
                show_confidence_band: true, 
                confidence_color_offset: 15,
                y_axis_label_size: 13,
                x_axis_label_size: 13,
                y_axis_title_size: 16,
                x_axis_title_size: 16,
                show_legend: true,
                legend_size: 14,
                smooth: false,
                marker_size: 4.0, 
            },
            evo: EvoConfig { 
                align: true, 
                align_origin: true, 
                t_max_diff: 0.01, 
                t_offset: 0.0, 
                scale: false,
                n_to_align: 100, 
            },
            rustle: RustleConfig { 
                start_offset: 5.0, //awaits 1s delay between start of algorithm and dataset play
                dataset_duration: -1.0, // Dataset duration, default is -1 to play the whole dataset
                dataset_start: 0.0,
                time_precision: 0.01, // Dataset start at N, default is None to start at the begining.
            },
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
