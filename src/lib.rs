use bollard::Docker;
use bollard::image::CreateImageOptions;
use bollard::container;
use bollard::exec::{CreateExecOptions, StartExecResults};
use bollard::models::{HostConfig,ResourcesUlimits};
use bollard::container::StatsOptions;

use std::{thread, time};

use tokio;
use tokio::select;
use tokio_util::sync::CancellationToken;

use std::error::Error;

use futures_util::stream::{StreamExt, TryStreamExt};

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
    
    //TODO refactor this
    let test = match "1"{
        "1" => Ok(()),
        _ => panic!("Test")
    };
    test
}

#[derive(Debug, Clone)]
pub struct Config {
    pub img_name: String,
    pub dataset_path: String,
    pub params_path: String,
    pub docker: Docker,
}
impl Config {
    // add code here
    pub async fn new (img_name: String, dataset_path: String, params_path: String) -> Result<Config, &'static str> {

        //TODO Check if img_name//dataset_path//params_path are valid.

        let docker = Docker::connect_with_local_defaults().unwrap();

        //Check if Docker Image is in the system, if not pull it from a Docker repository
        match docker.inspect_image(&img_name).await {
            Ok(_) => info!("Docker image found!"),
            Err(error) => match error{
                DockerResponseServerError => match pull_image(&img_name, &docker).await {
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
        let mut path_mounts = vec![];
        path_mounts.push(format!(
            "{}:{}",
            config.dataset_path,
            "/rustle/dataset/"
        ));

        path_mounts.push(format!(
            "{}:{}",
            config.params_path,
            "/rustle/config/"
        ));
        // refactor

        let config_docker = container::Config {
            image: Some(config.img_name.clone()),
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
        
        //Create the Execution instructions
        let play_options_future = self.config.docker
            .create_exec(
            &self.image_id,
            CreateExecOptions {
                attach_stdout: Some(true),
                attach_stderr: Some(true),
                cmd: Some(vec!["/bin/bash", "-l", "-c", "rosbag play -d 5 -u 10 /rustle/dataset/*.bag"]),
                ..Default::default()
            },
        );

        let task_options_future = self.config.docker
            .create_exec(
            &self.image_id,
            CreateExecOptions {
                attach_stdout: Some(true),
                attach_stderr: Some(true),
                cmd: Some(vec!["/bin/bash", "-l", "-c", "roslaunch rustle rustle.launch --wait"]),
                ..Default::default()
            },
        );
        
        // Check how I'm gonna record the localization
        // let record_options_future = self.docker
        //     .create_exec(
        //     &id,
        //     CreateExecOptions {
        //         // attach_stdout: Some(true),
        //         attach_stderr: Some(true),
        //         cmd: Some(vec!["/bin/bash", "-l", "-c", "rosbag record /rustle/result.bag -a"]),
        //         ..Default::default()
        //     },
        // );

        let play_exec_id = play_options_future.await.unwrap().id; 
        let task_exec_id = task_options_future.await.unwrap().id; 
        // let record_exec_id = record_exec_options.await.unwrap().id; 

       
        // Spawn a tokio::task (green thread) for each command to run, namely:
        //     - rosbag Play
        //     - roslaunch rustle
        //     - rosbag record
        //     - stream container data

        let docker1 = self.config.docker.clone();
        let token = CancellationToken::new();
        let cloned_token = token.clone();

        //spawn a task for roslaunch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = docker1.start_exec(&task_exec_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                            trace!("RUSTLE TASK: {msg}");
                        },
                        _ = cloned_token.cancelled()=>{
                            info!("Roslaunch was asked to stop");
                            //logic to cancel this task
                            break;
                        }
                    }
                }
            } else {
                warn!("STREAM ENDED");
                unreachable!();
            }

        });

        let docker2 = self.config.docker.clone();

        // Wait a certain number of seconds for the algorithm to init, PARAMETER
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        //spawn a task for roslaunch
        let rosplay_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = docker2.start_exec(&play_exec_id, None).await.unwrap() {
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
        // tokio::join!(roslaunch_task);



        //Create a thread to stream data
        // let stream = &mut self.config.docker
        //             .stats(
        //                 &self.image_id,
        //                 Some(StatsOptions {
        //                     stream: true,
        //                     one_shot: false
        //                 }),
        //             );
        //             // .take(1);
        //
        // while let Some(Ok(stats)) = stream.next().await {
        //     println!(
        //         "rustle-task -  {:?}",
        //         stats
        //     );
        // }



        


        //Stream container data (CPU load, memory, etc...)
        //Save the rosbag into a local object (or save it in a database?)
        //Return the saved trajectory//id in database.
        //Remove the container
    }
}
