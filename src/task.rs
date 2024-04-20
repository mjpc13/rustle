use bollard::{container::{self, RemoveContainerOptions, StatsOptions}, exec::{CreateExecOptions, StartExecResults}, image::CreateImageOptions, models::{HostConfig, ResourcesUlimits}, Docker
};

use std::{
    collections::HashMap, 
    error::Error, 
    fmt, 
    fs::{File, OpenOptions}, 
    io::Write, path::Path, 
    sync::{Arc, Mutex}, 
    thread, 
    time
};

use tokio;
use tokio::select;
use tokio_util::sync::CancellationToken;

use futures_util::stream::{StreamExt};
use futures_util::future;
use temp_dir::TempDir;

use crate::{db::DB, metrics::ContainerStats, ros_msgs::{GeometryMsg, PoseStamped, RosMsg}};
//use crate::metrics::Stats;

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
pub struct AdvancedConfig{
    pub docker_socket: Docker,
    pub db_endpoint: String,
    pub db_namespace: String,
    pub db_database: String
}

impl Default for AdvancedConfig {
    fn default() -> AdvancedConfig {
        AdvancedConfig{
            docker_socket: Docker::connect_with_local_defaults().unwrap(),
            db_endpoint: "memory".to_string(),
            db_namespace: "namespace".to_string(),
            db_database: "database".to_string()
        }
    }
}

#[derive(Debug, Clone)]
struct InnerConfig {
    img_name: String,
    algo_name: String,
    dataset_path: String,
    params_path: String,
    docker: Docker,
    topics: Vec<String>, //Should be &str, but lifetime issues, look at this later
    groundtruth: String, //Should be &str, but lifetime issues, look at this later
    db: DB,
    dir: TempDir
}

/// Wraps InnerConfig in an `Arc<InnerConfig>`
#[derive(Clone)]
pub struct Config {
    config: Arc<InnerConfig>
}
#[derive(Debug)]
pub struct TaskOutput<T: GeometryMsg> {
    pub stats: Vec<ContainerStats>,
    pub odoms: HashMap<String, Vec<Odometry>>,
    pub groundtruth: Vec<T>
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
    pub async fn new (img_name: String, algo_name: String, dataset_path: String, params_path: String, topics: Vec<String>, gt_topic: String, advanced_args: Option<AdvancedConfig>) -> Result<Config, &'static str> {
        // TODO: Change the return to Result<Config, RosError>

        //Creates the DB
        let connection = match advanced_args{
            Some(ref val) => {
                let connection = any::connect(&val.db_endpoint).await.unwrap();
                connection.use_ns(&val.db_namespace).use_db(&val.db_database).await.unwrap();
                connection
            } ,
            None => {
                let endpoint = std::env::var("SURREALDB_ENDPOINT").unwrap_or_else(|_| "memory".to_owned());
                let connection = any::connect(endpoint).await.unwrap();
                connection.use_ns("namespace").use_db(&algo_name).await.unwrap();
                connection
            }
        };

        let db = DB { db: connection };

        //TODO: Check if img_name//dataset_path//params_path are valid.
        let docker = match advanced_args{
            Some(val) => val.docker_socket,
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

        let dir = TempDir::new().unwrap();

        let config: InnerConfig = InnerConfig{
            img_name, 
            algo_name, 
            dataset_path, 
            params_path, 
            topics, 
            groundtruth: gt_topic, 
            docker: docker, 
            db, 
            dir
        };

        let config_arc = Arc::new(config);

        Ok(Config{config: config_arc})
    }

    /// Starts a container based on the img_name field.
    async fn start_container(&self) -> () {
        self.config.docker.start_container::<String>(self.get_img(), None).await;
    }

    fn get_img(&self) -> &str {
        &self.config.img_name
    }
    fn get_algo(&self) -> &str {
        &self.config.algo_name
    }
    fn get_gt(&self) -> &str {
        &self.config.groundtruth
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
        
        //TODO Parse the parameters of yaml? Or mount the YAML file into Docker container 
        let options = Some(container::CreateContainerOptions{
            name: format!("rustle-{}", config.get_algo()),
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

    pub async fn run(&self) -> Result<TaskOutput<PoseStamped>, RosError>{
        //Start the container
        // let _ = self.config.start_container().await;
        let _ = self.config.get_docker().start_container::<String>(&self.image_id, None).await;

        //Wait 1s for roscore to start
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let commands: Vec<_> = vec![
            "roslaunch rustle rustle.launch --wait",
            "rosbag play --clock /rustle/dataset/*.bag"
            //"rosbag play -d 0 --clock -u 20 /rustle/dataset/*.bag"
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

                        let stats = ContainerStats {
                            uid: Some(count),
                            memory_stats: msg.memory_stats,
                            cpu_stats: msg.cpu_stats,
                            precpu_stats: msg.precpu_stats,
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

        //Add the ground truth as a topic
        let mut topics = self.config.get_topics();
        topics.push(self.config.get_gt().to_string());
        
        let res_tasks: Vec<_> = topics
            .into_iter()
            .map( |s| {
                let config_clone = self.config.clone();
                let token = token.clone();
                tokio::spawn(async move {
                    Self::record_task(config_clone, &s, token).await;
                })
            })
            .collect();

        // Wait a certain number of seconds for the algorithm to init, PARAMETER
        let ten_sec = time::Duration::from_secs(5);
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

        let mut odoms_result = Arc::new(Mutex::new(HashMap::<String, Vec<Odometry>>::new()));
        let stats_result: Vec<ContainerStats> = self.config.get_db().query_stats().await.unwrap();

        let  gt_result: Vec<PoseStamped> = self.config.get_db().query_msg(self.config.get_gt()).await.unwrap();
        
        let write_results:Vec<_> = self.config.get_topics()
            .into_iter()
            .map(|s|{
                let config_clone = self.config.clone();
                let mut odoms_result_clone = odoms_result.clone();
                tokio::spawn(async move {
                    let odoms: Vec<Odometry> = config_clone
                        .get_db()
                        .query_msg(&s)
                        .await
                        .expect("Unable to find odometry data on table!");
                    let test = odoms_result_clone.lock().unwrap().insert(s.to_string(), odoms);
                })
            })
            .collect();




        //let write_results: Vec<_> = rec_cmd
        //    .into_iter()
        //    .map( |s| {
        //        let config_clone = self.config.clone();
        //        tokio::spawn(async move {
        //            let odoms: Vec<Odometry> = config_clone.get_db().query_odom(s).await.expect("Unable to find odometry data on table!");
        //            Self::write_result(odoms, s, config_clone.get_dir());
        //        })
        //    })
        //    .collect();

        
        self.config.get_docker().remove_container(
            &format!("rustle-{}", self.config.get_algo()), 
            Some(
                RemoveContainerOptions{
                    force: true,
                    ..Default::default()
                }
            )
        );
        
        futures_util::future::join_all(write_results).await;

        let odoms_result = Arc::into_inner(odoms_result).unwrap().into_inner().unwrap();

        Ok(TaskOutput{
            stats: stats_result,
            odoms: odoms_result,
            groundtruth: gt_result
        })

    }

    async fn record_task(config: Config, topic: &str, record_token: CancellationToken){

        let cmd = format!("rostopic echo -p {topic}");

        let image_id = format!("rustle-{}", config.get_algo());

         // Check how I'm gonna record the localization
        let record_options_future = config.get_docker()
            .create_exec(
            &image_id,
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

                            //TODO: CHECK THIS FUTURE MARIO
                            match Self::convert_to_ros::<Odometry>(msg.to_string()){
                                Ok(odom) => {
                                    config.get_db().add_msg(odom, topic).await;
                                },
                                Err(error) => {
                                    match Self::convert_to_ros::<PoseStamped>(msg.to_string()){
                                        Ok(pose_stamped) => {
                                            config.get_db().add_msg(pose_stamped, topic).await;
                                        },
                                        Err(e) => warn!("{e:}"),
                                    }
                                },
                            }
                        },
                        _ = record_token.cancelled()=>{
                            
                            let record_options_future = config.get_docker()
                                .create_exec(
                                &image_id,
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

    fn convert_to_ros<T: RosMsg>(msg: String) -> Result<T, RosError> {

        let ros_msg: Vec<_> = msg.split("\n")
            // .filter(|&s| s.chars().next().map(char::is_numeric).unwrap_or(false))
            .collect();

        if !ros_msg.is_empty() {
            
            let message_list: Vec<_> = ros_msg
                .iter()
                .next().expect("Was empty")
                .split(",")
                .collect();

            if message_list.len() >= 3{
                //TODO: Change this "default stuff"
                let res: T = T::from_vec(message_list)?;
                trace!("A valid ROS msg: {}", res.echo());

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
