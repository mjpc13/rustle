//Allow for user to use YAML as config options... Move the Config logic from task.rs to here!!!!
use bollard::{container::{self, RemoveContainerOptions, StatsOptions}, exec::{CreateExecOptions, StartExecResults}, image::{self, CreateImageOptions}, models::{HostConfig, ResourcesUlimits}, Docker
};
use serde::{Deserialize, Serialize};
use yaml_rust2::{parser::Parser, YamlLoader};

use std::{
    cmp::Ordering, collections::HashMap, error::Error, fmt, fs::{File, OpenOptions}, io::Write, path::Path, sync::Arc, task, thread, time
};

use tokio;
use tokio::select;
use tokio_util::sync::CancellationToken;
use tokio::sync::Mutex;

use futures_util::stream::{StreamExt};
use futures_util::future;
use temp_dir::TempDir;

use crate::{db::DB, metrics::ContainerStats, ros_msgs::{PoseStamped, RosMsg}};
//use crate::metrics::Stats;

use surrealdb::{
    Surreal,
    engine::any
};

use crate::errors::RosError;
use crate::ros_msgs::{Header, Pose, Twist, Odometry};

use rand::Rng;


//Logs
use log::{debug, error, info, warn, trace};

pub async fn pull_image(img_name: &str, docker: &Docker) -> Result<(), Box<dyn Error>>{

    let options = Some(CreateImageOptions{
      from_image: img_name,
      ..Default::default()
    });

    trace!("Docker Image downloading:");

    let mut image_down_stream = docker.create_image(options, None, None);

    while let Some(image_info) = image_down_stream.next().await {
        match image_info{
            Ok(msg) => trace!("{0:#?}", msg),
            Err(e) => panic!("Error downloading image: {:?}", e)
        }
    };
    
    //TODO: refactor this
    let test = match "1"{
        "1" => Ok(()),
        _ => panic!("Error downloading image")
    };
    test
}



#[derive(Debug, Clone)]
pub struct AdvancedConfig{
    pub docker_socket: Docker,
    pub db_connection: Surreal<surrealdb::engine::any::Any>,
}

#[derive(Debug, Clone)]
struct InnerConfig {
    img_name: String,
    algo_name: String,
    dataset_path: String,
    params_path: String,
    speed: u16,
    docker: Docker,
    topics: Vec<String>, //Should be &str, but lifetime issues, look at this later
    groundtruth: String, //Should be &str, but lifetime issues, look at this later
    db: DB,
    dir: TempDir
}

impl InnerConfig {
    
    pub fn set_speed(&mut self, new_speed: u16){
        self.speed = new_speed;
    }
}

/// Wraps InnerConfig in an `Arc<InnerConfig>`
#[derive(Debug, Clone)]
pub struct Config {
    config: Arc<InnerConfig>
}

impl Config {
    /// Constructs a new `Config`.
    /// 
    /// You can optionally pass a custom Docker socket (to work with rootless docker for instance) and a 
    /// custom endpoint. By default the database create is not persistent (is stored in memory), if you want to access the database after
    /// the execution of the program you should pass a custom endpoint.
    /// 
    /// # Examples
    /// ```
    /// let dataset_path = "/path/to/dataset/";
    /// let params_path = "/path/to/config/";
    /// let image_name = "mjpc13/rustle:lio-sam".into();
    /// let topics: Vec<String> = vec![
    ///    "/lio_sam/mapping/odometry".to_string(),
    /// ];
    /// let config = Config::new(
    ///        image_name,
    ///        dataset_path.into(),
    ///        params_path.into(),
    ///        topics,
    ///        None,
    ///        Some("file://path/to/database") //custom endpoint, this will write a persistent database
    ///    ).await;
    /// ```
    pub async fn new (img_name: &str, algo_name: &str, dataset_path: &str, params_path: &str, topics: Vec<&str>, gt_topic: &str, advanced_args: Option<&AdvancedConfig>, speed: Option<u16>) -> Result<Config, &'static str> {
        // TODO: Change the return to Result<Config, RosError>

        //Creates the DB
        let connection = match advanced_args{
            Some(val) => {

                let connection = val.db_connection.clone();
                //connection.use_ns(algo_name).use_db([task-name]).await.unwrap();
                info!("DB Connection: Custom");
                Arc::new(Mutex::new(connection))

            },
            None => {
                let endpoint = std::env::var("SURREALDB_ENDPOINT").unwrap_or_else(|_| "memory".to_owned());
                let connection = any::connect(endpoint).await.unwrap();
                //connection.use_ns("namespace").use_db(algo_name).await.unwrap();
                info!("DB Connection: Memory");
                Arc::new(Mutex::new(connection))
            }
        };

        let db = DB { db: connection };

        //TODO: Check if img_name//dataset_path//params_path are valid.
        let docker = match advanced_args{
            Some(val) => {
                info!("Docker Socket: Custom");
                val.docker_socket.clone()
            },
            None => {
                info!("Docker Socket: Default");
                Docker::connect_with_local_defaults().unwrap()
            }
        };

        let topics: Vec<String> = topics.into_iter().map(|s| s.to_string()).collect();

        //Check if Docker Image is in the system, if not pull it from a Docker repository
        match docker.inspect_image(&img_name).await {
            Ok(_) => info!("Docker image: Found"),
            Err(error) => match error{
                _DockerResponseServerError => match pull_image(&img_name, &docker).await {
                    Ok(_) => info!("Docker image: Pulled"),
                    Err(e) => panic!("Problem downloading image: {:?}", e),
                },
                other_error => {
                    panic!("Problem getting the Docker image: {:?}", other_error);
                }
            }
        };

        let dir = TempDir::new().unwrap();

        let speed = match speed {
            Some(s) => s,
            None => 1
        };

        let config: InnerConfig = InnerConfig{
            img_name: img_name.into(), 
            algo_name: algo_name.into(), 
            dataset_path: dataset_path.into(), 
            params_path: params_path.into(), 
            topics, 
            speed,
            groundtruth: gt_topic.into(), 
            docker: docker, 
            db, 
            dir
        };

        let config_arc = Arc::new(config);

        debug!("Config created: {:#?}", config_arc);

        Ok(Config{config: config_arc})
    }

    // Starts a container based on the img_name field.
    async fn start_container(&self) -> () {
        self.config.docker.start_container::<String>(self.get_img(), None).await;
    }

    pub fn get_img(&self) -> &str {
        &self.config.img_name
    }
    pub fn get_algo(&self) -> &str {
        &self.config.algo_name
    }
    pub fn get_gt(&self) -> &str {
        &self.config.groundtruth
    }
    pub fn get_dataset(&self) -> &str {
        &self.config.dataset_path
    }
    pub fn get_params(&self) -> &str {
        &self.config.params_path
    }
    pub fn get_docker(&self) -> &Docker {
        &self.config.docker
    }
    pub fn get_db(&self) -> &DB {
        &self.config.db
    }
    pub fn get_dir(&self) -> &TempDir {
        &self.config.dir
    }
    pub fn get_topics(&self) -> Vec<String> {
        self.config.topics.clone()
    }

    pub fn get_speed(&self) -> u16 {
        self.config.speed.clone()
    }

    // Setter for speed
    pub fn set_speed(&mut self, new_speed: u16) {
        // Use Arc::make_mut to get a mutable reference to InnerConfig
        Arc::make_mut(&mut self.config).set_speed(new_speed);
    }
}
