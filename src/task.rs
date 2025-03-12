use bollard::{container::{self, RemoveContainerOptions, StatsOptions}, exec::{CreateExecOptions, StartExecResults}, image::{self, CreateImageOptions}, models::{HostConfig, ResourcesUlimits}, Docker
};
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

async fn pull_image(img_name: &str, docker: &Docker) -> Result<(), Box<dyn Error>>{

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
    pub db_database: String
}

//impl Default for AdvancedConfig {
//    fn default() -> AdvancedConfig {
//        AdvancedConfig{
//            kc: Docker::connect_with_local_defaults().unwrap(),
//            db_connection: any::connect("memory").unwrap(),
//            db_database: "database".to_string()
//        }
//    }
//}

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
#[derive(Debug, Clone)]
pub struct Config {
    config: Arc<InnerConfig>
}
#[derive(Debug, Clone)]
pub struct TaskOutput{
    pub stats: Vec<ContainerStats>,
    pub odoms: HashMap<String, Vec<Odometry>>,
    pub groundtruth: Vec<Odometry>,
    pub name: String
}

// Implement `PartialEq` for equality comparison
impl PartialEq for TaskOutput {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

// Implement `Eq`
impl Eq for TaskOutput {}

// Implement `PartialOrd` for partial ordering
impl PartialOrd for TaskOutput {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Implement `Ord` for total ordering
impl Ord for TaskOutput {
    fn cmp(&self, other: &Self) -> Ordering {
        self.name.cmp(&other.name) // Sort by `name` field
    }
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
    pub async fn new (img_name: &str, algo_name: &str, dataset_path: &str, params_path: &str, topics: Vec<&str>, gt_topic: &str, advanced_args: Option<&AdvancedConfig>) -> Result<Config, &'static str> {
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

        let config: InnerConfig = InnerConfig{
            img_name: img_name.into(), 
            algo_name: algo_name.into(), 
            dataset_path: dataset_path.into(), 
            params_path: params_path.into(), 
            topics, 
            groundtruth: gt_topic.into(), 
            docker: docker, 
            db, 
            dir
        };

        let config_arc = Arc::new(config);

        debug!("Config created: {:#?}", config_arc);

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
    pub fn get_dir(&self) -> &TempDir {
        &self.config.dir
    }
    fn get_topics(&self) -> Vec<String> {
        self.config.topics.clone()
    }
}

pub struct Task{
    pub task_id: String,
    pub config: Config,
}

impl Task {
    // add code here
    // pub async fn new (config: Config) -> Result<Task, &'static str> {
    pub async fn new(config: Config) -> Task {

        //Create a random container name, unique for each task
        let mut rng = rand::thread_rng();
        let n1: u16 = rng.gen();
        let container_id = format!("rustle-{}-{}", config.get_algo(), n1);
        
        //TODO Parse the parameters of yaml? Or mount the YAML file into Docker container 
        let options = Some(container::CreateContainerOptions{
            name: &container_id,
            platform: None,
        });

        //Set up the binds config to mount these volumes inside our container
        let hm: HashMap<&str, &str> = HashMap::from([
            ("/rustle/dataset/", config.get_dataset()),
            ("/rustle/config/params.yaml", config.get_params()),
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
        

        //Setup container flags (cmd to execute, env variables, volumes to mount, etc...).
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

        config.get_docker().create_container(options, config_docker).await.unwrap();

        //debug!("Container created: {:#?}", id);

        Task{task_id: container_id, config}
    }

    pub async fn run(&self) -> Result<TaskOutput, RosError>{
        //Start the container
        debug!("Starting container: {:#?}", self.task_id);
        let _ = self.config.get_docker().start_container::<String>(&self.task_id, None).await;

        //Wait 1s for roscore to start
        //let ten_sec = time::Duration::from_secs(1);
        //thread::sleep(ten_sec);

        //Vector of commands to run inside the container
        let commands: Vec<_> = vec![
            "roslaunch rustle rustle.launch --wait",
            //"rosbag play --clock /rustle/dataset/*.bag"
            "rosbag play -d 9 --clock -u 40 /rustle/dataset/*.bag"
        ];
        let execs: Vec<_> = future::try_join_all(commands
            .iter()
            .map(|command| async {
                self.config.get_docker()
                    .create_exec(
                        &self.task_id,
                        CreateExecOptions {
                            attach_stdout: Some(true),
                            attach_stderr: Some(true),
                            cmd: Some(vec!["/bin/bash", "-l", "-c", command]),
                            ..Default::default()
                        },
                    ).await
            })).await.unwrap();


        //Create a cancellation token to stop the execution threads
        let token = CancellationToken::new();

        //Wait 1s for roscore to start
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let config_clone = self.config.clone();
        let roslaunch_id = execs[0].id.clone();
        let task_token = token.clone();
        let id = self.task_id.clone();

        //spawn a task for roslaunch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, mut input } = config_clone.get_docker().start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                            trace!("ROS MSG: {msg}");
                        },
                        _ = task_token.cancelled()=>{
                            info!("Container Stopped");
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

                            config_clone.get_docker().start_exec(&record_options_future.id, None).await.unwrap(); //This kills the record command
                            //the record command
                            debug!("Record command killed");
                            break;
                        }
                    }
                }
            } else {
                warn!("STREAM ENDED");
                unreachable!();
            }
        });




        //Add the ground truth as a topic, because we need to record it. Add option to provide a file directly
        let mut topics = self.config.get_topics();
        topics.push(self.config.get_gt().to_string());
        
        let res_tasks: Vec<_> = topics
            .into_iter()
            .map( |s| {
                let config_clone = self.config.clone();
                let token = token.clone();
                let task_id_clone = self.task_id.clone();

                tokio::spawn(async move {
                    Self::record_task(config_clone, task_id_clone, &s, token).await;
                })

            })
            .collect();

        // Wait a certain number of seconds for the algorithm to init, THIS SHOULD BE A PARAMETER
        let ten_sec = time::Duration::from_secs(5);
        thread::sleep(ten_sec);

        let task_id_clone = self.task_id.clone();
        let stream_token = token.clone();
        let config_clone = self.config.clone();

        let stream_task = tokio::spawn(async move{
            let stream= &mut config_clone.get_docker()
                    .stats(
                        &task_id_clone,
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
                        config_clone.get_db().add_stat(config_clone.get_algo(),&task_id_clone, stats).await;
                    },
                    _ = stream_token.cancelled()=>{
                        break;
                    }

                }
            }
        });


        let rosplay_id = execs[1].id.clone();
        let config_clone = self.config.clone();
        let task_id_clone = self.task_id.clone();

        //spawn a task for rosplay
        let rosplay_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = config_clone.get_docker().start_exec(&rosplay_id, None).await.unwrap() {
                while let Some(Ok(msg)) = output.next().await {
                    trace!("ROSBAG: {msg}");
                }
            } else {
                warn!("STREAM PLAY ENDED");
                unreachable!();
            }
                //Loop to give the prints;
        });

        
        tokio::join!(rosplay_task);

        debug!("Stopped rosbag play, send cancel signal to the other tasks");
        token.cancel(); // the end of the rosbag will be the first point where the other tasks need
        // to stop
        tokio::join!(roslaunch_task);
        debug!("Stopped roslaunch");

        let mut odoms_result = Arc::new(Mutex::new(HashMap::<String, Vec<Odometry>>::new()));
        let stats_result: Vec<ContainerStats> = self.config.get_db().query_stats(self.config.get_algo(), &self.task_id).await.unwrap();

        let  gt_result: Vec<Odometry> = self.config.get_db().query_odom(self.config.get_algo(), &self.task_id, self.config.get_gt()).await.unwrap();
        
        let write_results:Vec<_> = self.config.get_topics()
            .into_iter()
            .map(|s|{
                let config_clone = self.config.clone();
                let task_id_clone = self.task_id.clone();
                let mut odoms_result_clone = odoms_result.clone();
                tokio::spawn(async move {
                    let odoms: Vec<Odometry> = config_clone
                        .get_db()
                        .query_odom(config_clone.get_algo(), &task_id_clone, &s)
                        .await
                        .expect("Unable to find odometry data on table!");
                    
                    if odoms.is_empty(){
                        warn!("Odometry is empty, {:} failed to estimate odometry probably due to a bad config setup.", config_clone.get_algo());
                    }
                    
                    let test = odoms_result_clone.lock().await.insert(s.to_string(), odoms);
                })
            })
            .collect();

        
        //Remove the containers
        self.config.get_docker().remove_container(
            &self.task_id,
            Some(
                RemoveContainerOptions{
                    force: true,
                    ..Default::default()
                }
            )
        ).await;
        debug!("Container removed");
        
        futures_util::future::join_all(write_results).await;

        let odoms_result = Arc::into_inner(odoms_result).unwrap().into_inner();

        let name = String::from(self.config.get_algo());

        Ok(TaskOutput{
            stats: stats_result,
            odoms: odoms_result,
            groundtruth: gt_result,
            name: name
        })

    }

    async fn record_task(config: Config, task_id: String, topic: &str, record_token: CancellationToken){

        let cmd = format!("rostopic echo {topic}");

         // Check how I'm gonna record the localization
        let record_options_future = config.get_docker()
            .create_exec(
            &task_id,
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

                            match Self::convert_to_ros(msg.to_string()){
                                Ok(r) => {
                                    let odom = r.as_odometry().unwrap();
                                    config.get_db().add_odom(config.get_algo(), &task_id, odom, topic).await;
                                }
                                Err(e) => {
                                    warn!("{e:}");
                                }
                            }

                        },
                        _ = record_token.cancelled()=>{
                            
                            let record_options_future = config.get_docker()
                                .create_exec(
                                &task_id,
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

    fn convert_to_ros(msg: String) -> Result<RosMsg, RosError> {


        let yaml = match YamlLoader::load_from_str(&msg.replace("\n---\n", "")){
            Ok(y) => y,
            Err(e) => {
                return Err(RosError::FormatError { name: msg.into() })
            },
        };
        
        let ros_msg = yaml[0].as_hash().unwrap();

        let top_fields: Vec<_> = ros_msg
            .keys()
            .into_iter()
            .map(|y|{
                y.as_str().unwrap()
            }).collect();

        let ros = RosMsg::new(top_fields)?;

        return ros.from_yaml(yaml[0].clone());
        
    }

}

#[derive(Debug)]
pub struct TaskBatch{
    pub configs: Vec<Config>,
    pub iterations: usize,
    pub workers: u8
}

impl TaskBatch {
    pub async fn run(&self) -> Result<HashMap<&str, Vec<TaskOutput>>, RosError>{
        let mut res:Vec<TaskOutput> = vec![];

        let mut res_hash: HashMap<&str, Vec<TaskOutput>> =  HashMap::new();

        //init new algorithms
        for c in &self.configs{
            res_hash.insert(c.get_algo(), Vec::new());
        }

        //wrap hash in a new arc mutex
        let mut_res: Mutex<HashMap<&str, Vec<TaskOutput>>> = Mutex::new(res_hash);

        //Repeat the configurations for the number of times specified in the batch size and wrapped in an arc mutex
        let mut test = self.configs.clone();


        let task_jobs: Vec<Config> = self.configs.clone().into_iter().cycle().take(self.configs.len() * self.iterations).collect();
        let task_jobs: Arc<Mutex<Vec<Config>>> = Arc::new(Mutex::new(task_jobs));

        while task_jobs.lock().await.len() != 0{

            let results = (0..self.workers).map(|_| async {

                let mut config: Option<Config> = None;
                config = task_jobs.lock().await.pop();

                if let Some(c) = config{

                    info!("Running task: {:#?}", &c);

                    let task: Task = Task::new(c.clone()).await;
                    let res = task.run().await.unwrap();

                    mut_res.lock().await.get_mut(c.get_algo()).unwrap().push(res);

               } else{
                    debug!("No more tasks to run");
               }
            });
            futures_util::future::join_all(results).await;
        }


        let res_hash = mut_res.into_inner();

        Ok(res_hash)
    } 
}