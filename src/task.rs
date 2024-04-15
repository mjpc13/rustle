use bollard::{Docker,
    image::{CreateImageOptions},
    container,
    exec::{CreateExecOptions, StartExecResults},
    models::{HostConfig, ResourcesUlimits},
    container::{StatsOptions}
};

use std::{
    collections::HashMap,
    sync::Arc,
    thread,
    time,
    error::Error,
    fmt,
    fs::{OpenOptions,File},
    io::Write,
    path::Path
};

use tokio;
use tokio::select;
use tokio_util::sync::CancellationToken;

use futures_util::stream::{StreamExt};
use futures_util::future;
use temp_dir::TempDir;

use crate::db::{DB, Stats};
use surrealdb::{
    Surreal,
    engine::any
};

use crate::errors::RosError;
use crate::ros_msgs::{Header, Pose, Twist, Odometry};


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
struct InnerConfig {
    img_name: String,
    dataset_path: String,
    params_path: String,
    docker: Docker,
    topics: Vec<String>, //Should be &str, but lifetime issues, look at this later
    db: DB,
    dir: TempDir
}

#[derive(Clone)]
pub struct Config {
    config: Arc<InnerConfig>
}



impl Config {

    // TODO: Change the return to Result<Config, RosError>
    pub async fn new (img_name: String, dataset_path: String, params_path: String, topics: Vec<String>, docker: Option<Docker>, db: Option<Surreal<any::Any>>) -> Result<Config, &'static str> {

        //TODO: Check if img_name//dataset_path//params_path are valid.
        let docker = match docker{
            Some(val) => val,
            None => Docker::connect_with_local_defaults().unwrap()
        };

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
        
        //Creates the DB
        let db = match db{
            Some(val) => DB{ db: val },
            None => {
                let endpoint = std::env::var("SURREALDB_ENDPOINT").unwrap_or_else(|_| "memory".to_owned());
                let d = any::connect(endpoint).await.unwrap();
                d.use_ns("namespace").use_db("database").await.unwrap();
                DB { db: d }
            }
        };

        let dir = TempDir::new().unwrap();

        let config: InnerConfig = InnerConfig{img_name, dataset_path, params_path, topics, docker: docker, db, dir};
        let config_arc = Arc::new(config);

        Ok(Config{config: config_arc})
    }

    async fn start_container(&self) -> () {
        self.config.docker.start_container::<String>(self.get_img(), None).await;
    }

    fn get_img(&self) -> &str {
        &self.config.img_name
    }
    fn get_dataset(&self) -> &str {
        &self.config.dataset_path
    }
    fn get_params(&self) -> &str {
        &self.config.params_path
    }
    fn get_docker(&self) -> &Docker {
        &self.config.docker
    }
    fn get_db(&self) -> &DB {
        &self.config.db
    }
    fn get_dir(&self) -> &TempDir {
        &self.config.dir
    }
    fn get_topics(&self) -> Vec<String> {
        self.config.topics.clone()
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
        let hm: HashMap<&str, &str> = HashMap::from([
            ("/rustle/dataset/", config.get_dataset()),
            ("/rustle/config/", config.get_params()),
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
            image: Some(config.get_img()),
            cmd: Some(vec!["roscore"]),
            host_config: Some(HostConfig {
                binds: Some(path_mounts),
                ulimits: Some(vec![ResourcesUlimits{name:Some("nofile".to_string()),soft:Some(1024), hard:Some(524288)}]),
                ..Default::default()
            }),
            ..Default::default()
        };

        // Create a container
        let id: String = config.config.docker
            .create_container(options, config_docker)
            .await.unwrap()
            .id;

        Task{image_id:id, config}
    }

    pub async fn run(&self){
        //Start the container
        // let _ = self.config.start_container().await;
        let _ = self.config.get_docker().start_container::<String>(&self.image_id, None).await;

        //Wait 1s for roscore to start
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let commands: Vec<_> = vec![
            "roslaunch rustle rustle.launch --wait",
            // "rosbag play --clock /rustle/dataset/*.bag"
            "rosbag play -d 5 --clock -u 20 /rustle/dataset/*.bag"
        ];

        let execs: Vec<_> = future::try_join_all(commands
            .iter()
            .map(|command| async {
                self.config.get_docker()
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

        let image_id = self.image_id.clone();
        let stream_token = token.clone();
        let config_clone = self.config.clone();

        let stream_task = tokio::spawn(async move{
            let stream= &mut config_clone.get_docker()
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
                        config_clone.get_db().add_stat(stats).await;
                    },
                    _ = stream_token.cancelled()=>{
                        break;
                    }

                }
            }
        });


        // let docker = self.config.docker.clone();
        let config_clone = self.config.clone();
        let roslaunch_id = execs[0].id.clone();
        let task_token = token.clone();
        let id = self.image_id.clone();
        //spawn a task for roslaunch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, mut input } = config_clone.get_docker().start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                            trace!("RUSTLE TASK: {msg}");
                        },
                        _ = task_token.cancelled()=>{
                            info!("Roslaunch was asked to stop");
                            //logic to cancel this task

                            let record_options_future = config_clone.get_docker()
                                .create_exec(
                                &id,
                                CreateExecOptions {
                                    attach_stdout: Some(true),
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["killall", "-SIGINT", "record"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

                            config_clone.get_docker().start_exec(&record_options_future.id, None).await.unwrap(); //kill
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
        //let rec_cmd = vec!["/lio_sam/mapping/odometry_incremental"];  
        
        
        
        let res_tasks: Vec<_> = self.config.get_topics()
            .into_iter()
            .map( |s| {
                //TODO: TO Arc<Mutex<T>> OR NOT Arc<Mutex<T>>?
                let config_clone = self.config.clone();
                let id = self.image_id.clone();
                let token = token.clone();
                tokio::spawn(async move {
                    Self::record_task(config_clone, &id, &s, token).await;
                })
            })
            .collect();
 
        // Wait a certain number of seconds for the algorithm to init, PARAMETER
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let rosplay_id = execs[1].id.clone();
        let config_clone = self.config.clone();

        //spawn a task for rosplay
        let rosplay_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = config_clone.get_docker().start_exec(&rosplay_id, None).await.unwrap() {
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

        
        let rec_cmd = vec!["/lio_sam/mapping/odometry_incremental"];
        let write_results: Vec<_> = rec_cmd
            .into_iter()
            .map( |s| {
                // let db_clone = db.clone();
                let config_clone = self.config.clone();
                tokio::spawn(async move {
                    let odoms: Vec<Odometry> = config_clone.get_db().query_odom(s).await.expect("Unable to find odometry data on table!");
                    Self::write_result(odoms, s, config_clone.get_dir());
                })
            })
            .collect();

        let result = futures_util::future::join_all(write_results).await;


        //Return the N results;

    }

    async fn record_task(config: Config, image_id: &str, topic: &str, record_token: CancellationToken){

        let cmd = format!("rostopic echo -p {topic}");

         // Check how I'm gonna record the localization
        let record_options_future = config.get_docker()
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

        if let StartExecResults::Attached { mut output, .. } = config.get_docker().start_exec(&record_exec_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {

                            //TODO: Add custom ROSError here
                            match Self::convert_to_odom(msg.to_string()){
                                Ok(odom) => config.get_db().add_odom(odom, topic).await,
                                Err(error) => warn!("{}", error),
                            }

                        },
                        _ = record_token.cancelled()=>{
                            
                            let record_options_future = config.get_docker()
                                .create_exec(
                                image_id,
                                CreateExecOptions {
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["pkill", "-SIGINT", "record"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();    // kill -SIGINT pid

                            config.get_docker().start_exec(&record_options_future.id, None).await.unwrap();

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

    fn convert_to_odom(msg: String) -> Result<Odometry, RosError> {

        let ros_msg: Vec<_> = msg.split("\n")
            // .filter(|&s| s.chars().next().map(char::is_numeric).unwrap_or(false))
            .collect();


        warn!("ROS Message: {:?}", ros_msg);
        if !ros_msg.is_empty() {
            
            let message_list: Vec<_> = ros_msg
                .iter()
                .next().expect("Was empty")
                .split(",")
                .collect();

            if message_list.len() >= 3{
                //TODO: Change this "default stuff"
                let odom: Odometry = Odometry::default();
                let res: Odometry = odom.parse(message_list)?;

                warn!("{}", res);

                return Ok(res)
            }

        }
        return Err(RosError::FormatError{name: msg.into()})

    }

    fn write_result(data: Vec<Odometry>, name: &str, tmp_dir: &TempDir){

        // let path_name = format!("{}/{}.txt", path, name.replace("/", "-"));

        let f = tmp_dir.child(format!("{}", name.replace("/", "-")));
        
        // if Path::new(&path_name).is_file() == false{
        //     File::create(&path_name);
        // }

        let mut file = OpenOptions::new()
            .write(true)
            .append(true)
            .create(true)
            .open(&f).unwrap();

        
        let _: Vec<_> = data.iter()
            .map(|o|{
                if let Err(e) = writeln!(file, "{}", o ){
                    eprintln!("Couldn't write to file: {}", e);
                }
            })
            .collect();
    }

}
