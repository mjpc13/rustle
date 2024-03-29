use bollard::Docker;
use bollard::image::CreateImageOptions;
use bollard::container;
use bollard::exec::{CreateExecOptions, StartExecResults};
use bollard::models::{HostConfig,ResourcesUlimits};
use bollard::container::StatsOptions;

use std::collections::HashMap;
use std::sync::Arc;

use std::{thread, time};

use tokio;
use tokio::select;
use tokio_util::sync::CancellationToken;

use std::error::Error;

use futures_util::stream::{StreamExt};
use futures_util::future;

use crate::db::{DB, Stats};
use surrealdb::{
    Surreal,
    engine::any
};


//Logs
use log::{debug, error, info, warn, trace};

async fn pull_image(img_name: &str, docker: &Docker) -> Result<(), Box<dyn Error>>{

    let options = Some(CreateImageOptions{
      from_image: img_name,
      ..Default::default()
    });

    info!("Docker Image downloading:");

    let mut image_down_stream = docker.create_image(options, None, None);

    while let Some(image_info) = image_down_stream.next().await {
        match image_info{
            Ok(msg) => debug!("{0:#?}", msg),
            Err(e) => panic!("Error downloading image: {:?}", e)
        }
    };
    
    //TODO: refactor this
    let test = match "1"{
        "1" => Ok(()),
        _ => panic!("Test")
    };
    test
}

#[derive(Debug, Clone)]
pub struct Config {
    pub img_name: Arc<str>,
    pub dataset_path: Arc<str>,
    pub params_path: Arc<str>,
    pub docker: Docker,
}
impl Config {
    // add code here
    pub async fn new (img_name: Arc<str>, dataset_path: Arc<str>, params_path: Arc<str>) -> Result<Config, &'static str> {

        //TODO: Check if img_name//dataset_path//params_path are valid.

        let docker = Docker::connect_with_local_defaults().unwrap();

        //Check if Docker Image is in the system, if not pull it from a Docker repository
        match docker.inspect_image(&img_name).await {
            Ok(_) => info!("Docker image found!"),
            Err(error) => match error{
                _DockerResponseServerError => match pull_image(&img_name, &docker).await {
                    Ok(_) => (),
                    Err(e) => panic!("Problem downloading image: {:?}", e),
                },
                other_error => {
                    panic!("Problem getting the Docker image: {:?}", other_error);
                }
            }
        };

        Ok(Config{img_name, dataset_path, params_path, docker})
    }

}

pub struct Task{
    pub image_id: String,
    pub config: Config,
}

impl Task {
    // add code here
    // pub async fn new (config: Config) -> Result<Task, &'static str> {
    pub async fn new (config: Config) -> Task {
        //TODO: Create a container by mounting the dataset into Docker and pass relevant parameters
        
        //TODO Parse the parameters of yaml? Or mount the YAML file into Docker container 
        let options = Some(container::CreateContainerOptions{
            name: "rustle-task",
            platform: None,
        });

        //Set up the binds config to mount these volumes inside our container (needs refactor)
        let hm: HashMap<&str, &Arc<str>> = HashMap::from([
            ("/rustle/dataset/", &config.dataset_path),
            ("/rustle/config/", &config.params_path),
        ]);
        let mut path_mounts = vec![];
        let _: Vec<_> = hm.
            iter().
            map(|(k, v)| {
                path_mounts.push(format!(
                    "{}:{}",
                    v,
                    k
                ));
            })
            .collect();
        
        //Create the running config (cmd to execute, env variables, volumes to mount, etc...)
        let config_docker = container::Config {
            image: Some(config.img_name.to_string()),
            cmd: Some(vec!["roscore".to_string()]),
            host_config: Some(HostConfig {
                binds: Some(path_mounts),
                ulimits: Some(vec![ResourcesUlimits{name:Some("nofile".to_string()),soft:Some(1024), hard:Some(524288)}]),
                ..Default::default()
            }),
            ..Default::default()
        };

        // Create a container
        let id: String = config.docker
            .create_container(options, config_docker)
            .await.unwrap()
            .id;

    Task{image_id:id, config}
    }

    pub async fn run(&self){
        //Start the container
        let _ = self.config.docker.start_container::<String>(&self.image_id, None).await;

        //Wait 1s for roscore to start
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        //TODO: redo how I am calling the database
        let endpoint = std::env::var("SURREALDB_ENDPOINT").unwrap_or_else(|_| "memory".to_owned());
        let db = any::connect(endpoint).await.unwrap();
        db.use_ns("namespace").use_db("database").await.unwrap();
        let db = DB { db };


        let commands: Vec<_> = vec![
            "roslaunch rustle rustle.launch --wait",
            "rosbag play --clock /rustle/dataset/*.bag"
            // "rosbag play -d 5 --clock -u 10 /rustle/dataset/*.bag"
        ];

        let execs: Vec<_> = future::try_join_all(commands
            .iter()
            .map(|command| async {
                self.config.docker
                    .create_exec(
                        &self.image_id,
                        CreateExecOptions {
                            attach_stdout: Some(true),
                            attach_stderr: Some(true),
                            cmd: Some(vec!["/bin/bash", "-l", "-c", command]),
                            ..Default::default()
                        },
                    ).await
            })).await.unwrap();


        let token = CancellationToken::new();

        let docker = self.config.docker.clone();
        let image_id = self.image_id.clone();
        let stream_token = token.clone();
        let db_clone = db.clone();
        let stream_task = tokio::spawn(async move{
            let stream= &mut docker
                    .stats(
                        &image_id,
                        Some(StatsOptions {
                            stream: true,
                            one_shot: false
                        }),
                    );
            let mut count=0;
            loop{
                count+=1;
                select!{
                    Some(Ok(msg)) = stream.next() => {

                        let stats = Stats {
                            id: Some(count.to_string()),
                            memory_stats: msg.memory_stats,
                            cpu_stats: msg.cpu_stats,
                            num_procs: msg.num_procs,
                            created_at: None
                        };

                        // Add stats the the database;
                        db_clone.add_stat(stats).await;
                    },
                    _ = stream_token.cancelled()=>{
                        break;
                    }

                }
            }
        });


        let docker = self.config.docker.clone();
        let roslaunch_id = execs[0].id.clone();
        let task_token = token.clone();
        let id = self.image_id.clone();
        //spawn a task for roslaunch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, mut input } = docker.start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                            trace!("RUSTLE TASK: {msg}");
                        },
                        _ = task_token.cancelled()=>{
                            info!("Roslaunch was asked to stop");
                            //logic to cancel this task

                            let record_options_future = docker
                                .create_exec(
                                &id,
                                CreateExecOptions {
                                    attach_stdout: Some(true),
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["killall", "-SIGINT", "record"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

                            docker.start_exec(&record_options_future.id, None).await.unwrap(); //kill
                            //the record command

                            let ten_sec = time::Duration::from_secs(1);
                            thread::sleep(ten_sec);

                            break;
                        }
                    }
                }
            } else {
                warn!("STREAM ENDED");
                unreachable!();
            }
        });

        //spawn a task to extract the results and add them to DB
        //TODO: Dont hardcode this
        let rec_cmd = vec!["/lio_sam/mapping/odometry", "/lio_sam/mapping/odometry_incremental"];  
        let res_tasks: Vec<_> = rec_cmd
            .iter()
            .map( |s| {
                //TODO: TO Arc<Mutex<T>> OR NOT Arc<Mutex<T>>?
                let docker = self.config.docker.clone();
                let id = self.image_id.clone();
                let s = s.clone();
                let token = token.clone();
                let db_clone = db.clone();
                tokio::spawn(async move {
                    Self::record_task(docker, &id, db_clone, s, token).await;
                })
            })
            .collect();
 
        let docker = self.config.docker.clone();

        // Wait a certain number of seconds for the algorithm to init, PARAMETER
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let rosplay_id = execs[1].id.clone();

        //spawn a task for rosplay
        let rosplay_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = docker.start_exec(&rosplay_id, None).await.unwrap() {
                while let Some(Ok(msg)) = output.next().await {
                    trace!("RUSTLE PLAY: {msg}");
                }
            } else {
                warn!("STREAM PLAY ENDED");
                unreachable!();
            }
                //Loop to give the prints;
        });

        tokio::join!(rosplay_task);

        token.cancel(); // the end of the rosbag will be the first point where the other tasks need
        // to stop
        tokio::join!(roslaunch_task);

    }

    async fn record_task(docker: Docker, image_id: &str, db: DB, topic: &str, record_token: CancellationToken){

        let cmd = format!("rostopic echo -p {topic}");

         // Check how I'm gonna record the localization
        let record_options_future = docker
            .create_exec(
            image_id,
            CreateExecOptions {
                attach_stdout: Some(true),
                attach_stderr: Some(true),
                cmd: Some(vec!["/bin/bash", "-l", "-c", &cmd]), //need a different way to pass the topics to record
                ..Default::default()
            },
        );

        let record_exec_id = record_options_future.await.unwrap().id; 

        if let StartExecResults::Attached { mut output, .. } = docker.start_exec(&record_exec_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {

                            //Split msg by "\n" if the second field is not empty than we are
                            //receiving the first message;

                            //TODO: Add odometry messages into the database
                            
                            // warn!("{}", msg)

                        },
                        _ = record_token.cancelled()=>{
                            
                            let record_options_future = docker
                                .create_exec(
                                image_id,
                                CreateExecOptions {
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["pkill", "-SIGINT", "record"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();    // kill -SIGINT pid

                            docker.start_exec(&record_options_future.id, None).await.unwrap();

                            let ten_sec = time::Duration::from_secs(5);
                            thread::sleep(ten_sec);

                            break;
                        }
                    }
                }
            } else {
                warn!("STREAM ENDED");
            }
    }
}
