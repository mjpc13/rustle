use std::{cmp::Ordering, collections::{hash_map::Entry, HashMap}, sync::Arc, thread, time};

use bollard::{container::{self, RemoveContainerOptions}, exec::{CreateExecOptions, StartExecResults}, secret::{HostConfig, ResourcesUlimits}};
use futures_util::{future, StreamExt};
use log::{debug, info, trace, warn};
use rand::Rng;
use tokio::{select, sync::Mutex};
use tokio_util::sync::CancellationToken;
use yaml_rust2::YamlLoader;

use crate::{config::Config, errors::{EvoError, RosError}, evo_wrapper::EvoApeArg, metrics::{self, Metric}, ros_msgs::{Odometry, RosMsg}, task::TaskOutput};


pub struct SpeedTaskOutput{
    task_output: TaskOutput,
    pub bag_speed: u16,
    pub freq_avg: HashMap<String, f32>,
    pub name: String,
    //pub ape: HashMap<String, Vec<Result<Metric, EvoError>>>,
    //pub rpe: Vec<RPE>,
}

// Implement `PartialEq` for equality comparison
impl PartialEq for SpeedTaskOutput {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
    }
}

// Implement `Eq`
impl Eq for SpeedTaskOutput {}

// Implement `PartialOrd` for partial ordering
impl PartialOrd for SpeedTaskOutput {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Implement `Ord` for total ordering
impl Ord for SpeedTaskOutput {
    fn cmp(&self, other: &Self) -> Ordering {
        self.name.cmp(&other.name) // Sort by `name` field
    }
}

pub struct Task{
    pub task_id: String,
    pub container_id: String,
    pub config: Config,
}


impl Task {
    pub async fn new(config: Config) -> Task {

        //Create a random container name, unique for each task
        let mut rng = rand::rng();
        let n1: u16 = rng.random();
        let task_id = format!("{n1}");
        let container_id = format!("rustle-speed-{}-{}", config.get_algo(), &task_id);
        
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
        Task{task_id, container_id, config}
    }


    pub async fn run(&self, bag_speed: u16) -> Result<SpeedTaskOutput, RosError>{
        //Start the container
        debug!("Starting container: {:#?}", self.container_id);
        let _ = self.config.get_docker().start_container::<String>(&self.container_id, None).await;


        //let cmd = format!("rosbag play -d 9 -r {} --clock -u 20 /rustle/dataset/*.bag", bag_speed);
        let cmd = format!("rosbag play -r {} --clock /rustle/dataset/*.bag", bag_speed);

        //Vector of commands to run inside the container
        let commands: Vec<_> = vec![
            "roslaunch rustle rustle.launch --wait",
            &cmd
        ];
        let execs: Vec<_> = future::try_join_all(commands
            .iter()
            .map(|command| async {
                self.config.get_docker()
                    .create_exec(
                        &self.container_id,
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
        let id = self.container_id.clone();

        //spawn a task for roslaunch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, mut input } = config_clone.get_docker().start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                           //trace!("ROS MSG: {msg}");
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
                                    cmd: Some(vec!["/bin/bash", "-l", "-c", "rosnode", "kill", "-a", "2>/dev/null"]),
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
                let container_id_clone = self.container_id.clone();
                let task_id_clone = self.task_id.clone();

                tokio::spawn(async move {
                    Self::record_task(config_clone, container_id_clone, task_id_clone, &s, token, bag_speed).await;
                })

            })
            .collect();

        // Wait a certain number of seconds for the algorithm to init, THIS SHOULD BE A PARAMETER
        let ten_sec = time::Duration::from_secs(5);
        thread::sleep(ten_sec);

        let rosplay_id = execs[1].id.clone();
        let config_clone = self.config.clone();
        let task_id_clone = self.container_id.clone();

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
        let mut freq_result = Arc::new(Mutex::new(HashMap::<String, f32>::new()));

        let  gt_result: Vec<Odometry> = self.config.get_db().query_odom1("SpeedTask", self.config.get_algo(), "groundtruth").await.unwrap();

        let write_results:Vec<_> = self.config.get_topics()
            .into_iter()
            .map(|s|{
                let config_clone = self.config.clone();
                let task_id_clone = self.task_id.clone();
                let mut odoms_result_clone = odoms_result.clone();
                tokio::spawn(async move {

                    let odoms: Vec<Odometry> = config_clone
                        .get_db()
                        .query_odom("SpeedTask", config_clone.get_algo(), &format!("{}_{}x_{}",config_clone.get_speed(), &s, &task_id_clone))
                        .await
                        .expect("Unable to find odometry data on table!");

                    let freqs = config_clone
                        .get_db()
                        .get_frequency_avg(config_clone.get_algo(), &task_id_clone, &s)
                        .await
                        .expect("Unable to find odometry data on table!");
                    
                    if odoms.is_empty() {
                        warn!("Odometry is empty, {:} failed to estimate odometry probably due to a bad config setup.", config_clone.get_algo());
                    }
                    
                    let test = odoms_result_clone.lock().await.insert(s.to_string(), odoms);
                })
            })
            .collect();
        
        //Remove the containers
        self.config.get_docker().remove_container(
            &self.container_id,
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
        let freq_avg = Arc::into_inner(freq_result).unwrap().into_inner();

        let name = String::from(self.config.get_algo());

        //Need to compute the APE//RPE for this algorithm...

        //let odoms_result: Vec<Odometry> = vec![];
        let ape = HashMap::<String, Metric>::new();

        let to = TaskOutput{
            stats: vec![],
            odoms: odoms_result,
            groundtruth: gt_result,
            name: name.clone()
        };

        // 

        Ok(SpeedTaskOutput{
            task_output: to,
            bag_speed: bag_speed,
            name,
            freq_avg,
            //rpe: Vec<RPE>,
        })

    }


    async fn record_task(config: Config, container_id: String, task_id: String, topic: &str, record_token: CancellationToken, speed: u16){

        let cmd = format!("rostopic echo {topic}");

         // Check how I'm gonna record the localization
        let record_options_future = config.get_docker()
            .create_exec(
            &container_id,
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
                                    if topic.eq(config.get_gt()){
                                        if speed == 1 {
                                            config.get_db().add_odom1("SpeedTask", config.get_algo(), odom, "groundtruth").await;
                                        }
                                    }
                                    else{
                                        config.get_db().add_odom1("SpeedTask", config.get_algo(), odom, &format!("{}x_{}_{}",speed, topic, &task_id)).await;
                                    }
                                }
                                Err(e) => {
                                    warn!("{e:}");
                                }
                            }

                        },
                        _ = record_token.cancelled()=>{
                            
                            let record_options_future = config.get_docker()
                                .create_exec(
                                &container_id,
                                CreateExecOptions {
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["/bin/bash", "-l", "-c", "rosnode", "kill", "-a", "2>/dev/null"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

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
pub struct SpeedTaskBatch{
    pub configs: Vec<Config>,
    pub speed_list: Vec<u16>,
    pub iterations: usize,
    pub workers: u8
}

impl SpeedTaskBatch {
    pub async fn run(&self){
        for speed in self.speed_list.clone(){

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

                        let algo = c.get_algo();
    
                        let task: Task = Task::new(c).await;
                        let res = task.run(speed).await.unwrap();
                        
                        //This needs to be SpeedTask
                        //mut_res.lock().await.get_mut(algo).unwrap().push(res.task_output);
    
                   } else{
                        debug!("No more tasks to run");
                   }
                });
                futures_util::future::join_all(results).await;
            }
            let res_hash = mut_res.into_inner();

            //Compute APE with the metrics stuff
            let batch_res = Metric::compute_batch(
                &res_hash, 
                EvoApeArg{
                    plot: None,
                    ..Default::default()
                }
            );


            println!("For the {speed} we have the following results:");
            //__PRINT_IN_MD_TABLE___
            let md_batch = Metric::print_batch(&batch_res);
            println!("{}\n ---------------------------------", md_batch);
            //______________________
        

        }

        //Ok(res_hash)
    } 
}