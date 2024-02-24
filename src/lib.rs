use bollard::Docker;
use bollard::image::CreateImageOptions;
use bollard::container;
use bollard::exec::{CreateExecOptions, StartExecResults};
use bollard::models::HostConfig;

use std::{thread, time};



use std::error::Error;

use futures_util::stream::{StreamExt, TryStreamExt};


async fn pull_image(img_name: &str, docker: &Docker) -> Result<(), Box<dyn Error>>{

    let options = Some(CreateImageOptions{
      from_image: img_name,
      ..Default::default()
    });

    println!("Docker Image downloading:");

    let mut image_down_stream = docker.create_image(options, None, None);

    while let Some(image_info) = image_down_stream.next().await {
        match image_info{
            Ok(msg) => println!("{0:#?}", msg),
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
            Ok(_) => println!("Docker image found!"),
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
                binds: Some(path_mounts),..Default::default()
            }),
            ..Default::default()
        };

        // Create a container
        let id: String = config.docker
            .create_container(options, config_docker)
            .await.unwrap()
            .id;

    println!("STOPING");

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
                // attach_stdout: Some(true),
                attach_stderr: Some(true),
                cmd: Some(vec!["/bin/bash", "-l", "-c", "rosbag play /rustle/dataset/*.bag"]),
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

        //Do I need a select? OR can I just stream data after this block?
        //How can I know that the record has stopped?

        self.config.docker.start_exec(&task_exec_id, None);
        thread::sleep(time::Duration::from_secs(5));
        self.config.docker.start_exec(&play_exec_id, None).await;



        //Stream container data (CPU load, memory, etc...)
        //Save the rosbag into a local object (or save it in a database?)
        //Return the saved trajectory//id in database.
        //Remove the container
    }
}
