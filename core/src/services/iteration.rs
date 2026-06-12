use std::fs;
use std::path::Path;
use std::str::FromStr;
use std::time::{SystemTime, UNIX_EPOCH};
use std::{collections::HashMap, fs::OpenOptions, path::PathBuf, sync::Arc, thread, time};
use std::io::Write;

use bollard::container::LogOutput;
use bollard::secret::PortBinding;
use bollard::{container::{self, RemoveContainerOptions, StatsOptions}, exec::{CreateExecOptions, StartExecResults}, secret::{HostConfig, ResourcesUlimits}, Docker};
use charming::Chart;
use chrono::{Utc};
use futures_util::{future, StreamExt, SinkExt};
use log::{debug, info, warn};
use rand::{rng, RngExt};
use rand::{distr::Alphanumeric, Rng};
use tokio::net::TcpStream;
use tokio::sync::mpsc::Sender;
use tokio::time::{sleep, timeout, Duration, Instant};
use tokio::{select};
use tokio_tungstenite::{MaybeTlsStream, connect_async, tungstenite};
use serde_json::{Value, json};

use surrealdb::sql::Thing;
use tokio_util::sync::CancellationToken;
use yaml_rust2::YamlLoader;

use crate::db::OdometryRepo;
use crate::models::metric::Metric;
use crate::models::metrics::memory::MemoryMetrics;
use crate::models::metrics::metric::StatisticalMetrics;

use crate::models::metrics::pose_error::{PoseErrorMetrics, Position, APE, RPE};
use crate::models::metrics::{ContainerStats, CpuMetrics};
use crate::models::{ProgressMessage, Dataset, TestDefinition, TestType};
use crate::services::params::ParamsService;
use crate::utils::config::Config;
use crate::utils::evo_wrapper::{run_metrics_py, EvoApeArg, EvoRpeArg, PlotArg};
use crate::utils::plots::{ape_line_chart, cpu_load_line_chart, memory_usage_line_chart, rpe_line_chart};
use crate::{
    db::{iteration::IterationRepo}, 
    models::{iteration::{DockerContainer, Iteration}, 
    ros::ros_msg::RosMsg, Algorithm, Odometry}, 
    services::error::{DbError, ProcessingError, RosError}, 
    utils::evo_wrapper::EvoArg
};

use directories::ProjectDirs;

use super::error::{EvoError, PlotError};
use super::{MetricService};
use super::{error::RunError, DatasetService, StatService};
#[derive(Clone)]
pub struct IterationService {
    repo: IterationRepo,
    pub config: Config,
    dataset_service: DatasetService,
    stat_service: StatService,
    metric_service: MetricService,
    params_service: ParamsService,
    odom_repo: OdometryRepo,
    docker: Arc<Docker>,  // Assuming you have Docker client setup
}

impl IterationService {
    pub fn new(repo: IterationRepo, odom_repo: OdometryRepo, docker: Arc<Docker>, dataset_service: DatasetService, stat_service: StatService, metric_service: MetricService, params_service: ParamsService) -> Self {
        let config = Config::load().expect("Missing Configuration");
        Self { repo, docker, dataset_service, stat_service, metric_service, config, params_service, odom_repo }
    }

    pub async fn create(&self, iter_number: u64, algo:Algorithm, algorithm_run_id: &Thing, exec_id: &Thing, test_type: TestType) -> Result<(), DbError> {
        
        let sanitized = algo.name.replace(|c: char| !c.is_alphanumeric(), "_")
        .to_lowercase();

        let rng = rng();
        let rand_str: String = rng
            .sample_iter(Alphanumeric)
            .take(8)
            .map(char::from)
            .collect();

        let container_name = format!("{}_{}_{}",sanitized, iter_number, rand_str);
        let docker_container = DockerContainer{
            image_name: algo.image_name,
            container_name,
        };

        let mut iteration = Iteration {
            id: None,
            iteration_num: iter_number,
            container: docker_container,
            created_at: Utc::now(),
            test_type,
            exec_id: exec_id.clone()
        };

        let _ = self.repo.save(&mut iteration, algorithm_run_id).await;
        Ok(())
    }

    pub async fn run(&self, iter: Iteration, msg_tx: Option<Sender<ProgressMessage>>) -> Result<(), RunError> {

        let algorithm = self.repo.get_algorithm(&iter).await.unwrap(); //Maybe wrap in an Arc<>, Also, maybe wrap iter in an Arc

        // -- CREATE THE CONTAINER ---
        let _ = self.start_container(&iter).await;

        //get the corresponding algorithm run
        let algorithm_run = self.repo.get_algorithm_run(&iter).await.unwrap();

        let dataset = self.repo.get_dataset(&iter).await.unwrap(); //get the dataset

        let cmd = match self.config.rustle.dataset_duration {
            -1.0 => format!("rosbag play -s {} -r {} --clock /rustle/dataset/*.bag", self.config.rustle.dataset_start, algorithm_run.bag_speed),
            _ => format!("rosbag play -s {} -u {} -r {} --clock /rustle/dataset/*.bag", self.config.rustle.dataset_start, self.config.rustle.dataset_duration, algorithm_run.bag_speed)
        };

        let rustle_cmd = format!("roslaunch rustle-ros rustle.launch --wait test_type:={} algo_topic:={} gt_topic:={}", &iter.test_type.as_str(), &algorithm.odom_topics[0], &dataset.ground_truth_topic.clone().unwrap());

        //Vector of commands to run inside the container
        let commands: Vec<_> = vec![
            &rustle_cmd,
            &cmd
        ];
        let execs: Vec<_> = future::try_join_all(commands
            .iter()
            .map(|command| async {
                self.docker
                    .create_exec(
                        &iter.container.container_name,
                        CreateExecOptions {
                            attach_stdout: Some(true),
                            attach_stderr: Some(true),
                            cmd: Some(vec!["/bin/bash", "-l", "-c", command]),
                            ..Default::default()
                        },
                    ).await
            })).await.unwrap();

        //Create a cancellation token to stop the execution threads
        let token = Arc::new(CancellationToken::new());

        //Wait 1s for roscore to start
        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let roslaunch_id = execs[0].id.clone();
        let task_token = token.clone();
        let docker_clone = self.docker.clone();
        let iter_clone = iter.clone();

        //Execute the rustle.launch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, input: _ } = docker_clone.start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(_msg)) = output.next() => {
                           //info!("ROS MSG: {_msg}");
                        },
                        _ = task_token.cancelled()=>{
                            //info!("Container Stopped");
                            //logic to cancel this task

                            let record_options_future = docker_clone
                                .create_exec(
                                &iter_clone.container.container_name,
                                CreateExecOptions {
                                    attach_stdout: Some(true),
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["/bin/bash", "-l", "-c", "rosnode", "kill", "-a", "2>/dev/null"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

                            docker_clone.start_exec(&record_options_future.id, None).await.unwrap(); //This kills the record command
                            //the record command
                            debug!("Record command killed");
                            break;
                        }
                    }
                }
            }
        });

        let topic_list = algorithm.odom_topics;
        
        //TODO: OPTIMIZE THIS, THINK OF A BETTER WAY TO SAVE THE GT TOPIC
        let _res_tasks: Vec<_> = topic_list
            .into_iter()
            .map( |s: String| {
                let iter_id_clone = iter.id
                .clone()  // Clone the Option first
                .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();

                let odom_repo_clone = self.odom_repo.clone();
                
                tokio::spawn(async move {
                    // LOGIC TO SAVE ODOMS TO DB 
                    let _ = Self::record_task_ws(&s, &iter_id_clone, odom_repo_clone).await;
                })
            })
            .collect();


       // Wait a certain number of seconds for the algorithm to init
       let ten_sec = time::Duration::from_secs(self.config.rustle.start_offset as u64);
       thread::sleep(ten_sec);

       //Start the STATS collection
        let task_id_clone = iter.container.container_name.clone();
        let iteration_id_clone = iter.id
            .clone()  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();       
        let stream_token = token.clone();

        let docker_clone = self.docker.clone();
        let stat_service_clone = self.stat_service.clone();

       let _stream_task = tokio::spawn(async move{
           let stream= &mut docker_clone
                   .stats(
                       &task_id_clone,
                       Some(StatsOptions {
                           stream: true,
                           one_shot: false
                       }),
                   );
           loop{
               select!{
                   Some(Ok(msg)) = stream.next() => {

                       let stats = ContainerStats::new(msg.memory_stats, msg.cpu_stats, msg.precpu_stats, msg.num_procs);

                       // Add stats the the database;
                       let _ = stat_service_clone.record_stats(stats, &iteration_id_clone).await;

                   },
                   _ = stream_token.cancelled()=>{
                       break;
                   }

               }
           }
       });

        let rosplay_id = execs[1].id.clone();
        let docker_clone = self.docker.clone();

        //Start the rosbag play
        let rosplay_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, .. } = docker_clone.start_exec(&rosplay_id, None).await.unwrap() {
                while let Some(Ok(log)) = output.next().await {
                    let line = match log {
                        LogOutput::StdOut { message } |
                        LogOutput::StdErr { message } |
                        LogOutput::Console { message } => {
                            String::from_utf8_lossy(&message).to_string()
                        },
                        _ => continue, // Ignore unknown log types
                    };
        
                    if let Some(progress) = parse_rosbag_line(&line, iter.iteration_num.into(), algorithm_run.to_string() ) {
                        if let Some(tx) = &msg_tx {
                            if tx.send(progress).await.is_err() {
                                warn!("Progress receiver dropped for iteration {}", iter.iteration_num);
                            }
                        } else {
                            info!(
                                "Iter {} Bag Time: {:.6}   Duration: {:.6} / {:.6}",
                                progress.iteration_num,
                                progress.bag_time,
                                progress.duration,
                                progress.total_duration
                            );
                        }
                    }
                }
            } else {
                warn!("STREAM PLAY ENDED unexpectedly for iteration {}", iter.iteration_num);
            }
        });

        let _ = tokio::join!(rosplay_task);
        token.cancel(); // the end of the rosbag will be the first point where the other tasks need
        // to stop
        let _ = tokio::join!(roslaunch_task);

        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let iteration_id_clone = iter.id
            .clone()  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();

        //CLEAN RESIDUAL CONTAINERS
        let _ = self.remove_container(&iter.container.container_name).await;

        //If the dataset duration was not set before set it now
        let _ = self.dataset_service.set_duration(&dataset).await;

        warn!("Computing frequency...");

        //Compute the frequency
        let freq = self.repo.get_odom_frequency(&iter).await.map_err(|_e| RunError::Evo("Unable to find odometries".to_owned()))?;
        
        let freq_metric = StatisticalMetrics::from_single_value(freq as f32);
        let _ = self.metric_service.create_freq_metric(iteration_id_clone.clone(), freq_metric).await; // add to DB

        warn!("Computing stats...");
        let stats = self.stat_service.get_stats(&iter).await.unwrap();
        
        warn!("Computing cpu...");
        //Compute CPU stats
        let cpu_metric_opt = CpuMetrics::from_stats(&stats);
        if let Some(cpu_metric) = cpu_metric_opt {
            let _ = self.metric_service.create_cpu_metric(iteration_id_clone.clone(), cpu_metric).await.unwrap();  
        }

        warn!("Computing memory...");
        //Compute Memory stats
        let memory_metric_opt = MemoryMetrics::from_stats(&stats).unwrap();
        if let Some(memory_metric) = memory_metric_opt {
            let _ = self.metric_service.create_memory_metric(iteration_id_clone.clone(), memory_metric).await.unwrap();  
        }

        //This needs to be reviewed
        warn!("Computing APE's...");
        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned()))?;


            //Create the folder for this iteration:
            let iter_path = self.get_parents_string(&iter).await?;
            let dataset_name = self.get_dataset_name(&iter).await?;

            let full_path = format!("{data_dir_str}/data/{iter_path}");
            let full_dataset_path = format!("{data_dir_str}/data/dataset_{dataset_name}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();

            warn!("Computing APE and RPE!");
            //Compute the APE and RPE metrics
            let ape_args = EvoApeArg{
                //plot: Some(PlotArg::default()),
                plot: Some(PlotArg{
                    path: full_path.clone(),
                    ..Default::default()
                }),
                t_max_diff: Some(self.config.evo.t_max_diff as f32),
                t_offset: Some(self.config.evo.t_offset as f32),
                align: self.config.evo.align,
                scale: self.config.evo.scale,
                n_to_align: Some(self.config.evo.n_to_align as f32),
                ..Default::default()
            };

            let rpe_args = EvoRpeArg{
                plot: Some(PlotArg{
                    path: full_path.clone(),
                    ..Default::default()
                }),
                t_max_diff: Some(self.config.evo.t_max_diff as f32),
                t_offset: Some(self.config.evo.t_offset as f32),
                align: self.config.evo.align,
                scale: self.config.evo.scale,
                n_to_align: Some(self.config.evo.n_to_align as f32),
                ..Default::default()
            };


            let _ape = self.compute_metrics(&iter, &ape_args, &full_path, &full_dataset_path).await.map_err(|e| RunError::Evo(format!("Failed to compute APE {e}")))?;
            let _rpe = self.compute_metrics(&iter, &rpe_args, &full_path, &full_dataset_path).await.map_err(|e| RunError::Evo(format!("Failed to compute RPE {e}")))?;

            warn!("Read from file!");
            let mut ape_list = APE::read_from_file(&format!("{full_path}/ape.txt")).map_err(|e| RunError::Evo("Failure to load poses".to_owned()))?;
            let mut rpe_list = RPE::read_from_file(&format!("{full_path}/rpe.txt")).map_err(|e| RunError::Evo("Failure to load poses".to_owned()))?;
            let mut position_list = Position::read_from_file(&format!("{full_path}/aligned_poses.txt")).unwrap();

            warn!("Create ape rpe position!");
            let _ = self.metric_service.create_ape(iteration_id_clone.clone(), &mut ape_list).await;
            let _ = self.metric_service.create_rpe(iteration_id_clone.clone(), &mut rpe_list).await;
            let _ = self.metric_service.create_position(iteration_id_clone.clone(), &mut position_list).await;

            let ape_values = ape_list.iter()
                .map(|ape|{
                    ape.value
                }).collect();
            let rpe_values = rpe_list.iter()
                .map(|rpe|{
                    rpe.value
                }).collect();

            warn!("Create metrics");
            let pose_error_metric = PoseErrorMetrics::from_values(&ape_values, &rpe_values).unwrap();
            let _metric = self.metric_service.create_pose_error_metric(iteration_id_clone.clone(), pose_error_metric).await.unwrap();

        };
        warn!("Finish iteration!");

        Ok(())
    }

    //Probably create a plot that receives a iteration ID? then gets whatever it needs...
    pub async fn plot(&self, iter: Iteration, test_type: &TestType, path: &str, overwrite: bool, format:  &str) -> Result<HashMap<String, Chart>, PlotError>{

        let mut hash_chart: HashMap<String, Chart> = HashMap::new();

        let iter_path = self.get_parents_string(&iter).await.unwrap();
        let full_path = format!("{path}/{iter_path}");

        //Create the directories if they dont exist
        fs::create_dir_all(&full_path).unwrap();

        // Call the other plots
        let files = ["cpu_load", "memory_usage", "ape", "rpe"];

        // Get stats data:
        let stats = self.get_stats(&iter).await.map_err(|_e| PlotError::MissingData(format!("Missing stats for iteration {:?}", iter.id)))?;
        let ape_data = self.get_ape(&iter).await.map_err(|_e| PlotError::MissingData(format!("Missing APE values for iteration {:?}", iter.id)))?;
        let rpe_data = self.get_rpe(&iter).await.map_err(|_e| PlotError::MissingData(format!("Missing RPE values for iteration {:?}", iter.id)))?;

        for f in files{

            let filepath = format!("{}/{}.{}", full_path, f, format);

            if Path::new(&filepath).exists() && !overwrite {
                //Not sure if I should return here, or just emit a warning
                return Err(PlotError::FileExists(filepath));
            } else {
                let chart = match f {
                    "cpu_load" => cpu_load_line_chart(&stats, &self.config),
                    "memory_usage" => memory_usage_line_chart(&stats, &self.config),
                    "ape" => ape_line_chart(&ape_data, test_type, &self.config),
                    "rpe" => rpe_line_chart(&rpe_data, test_type, &self.config),
                    &_ => todo!("This should be fine")
                };
                hash_chart.insert(filepath, chart?);
            }

        }

        Ok(hash_chart)
    }

    pub async fn get_metrics(&self, iter: &Iteration) -> Result<Vec<Metric>, DbError>{

        let metrics = self.repo.get_metrics(iter).await?;
        Ok(metrics)
    }
    
    pub async fn get_stats(&self, iter: &Iteration) -> Result<Vec<ContainerStats>, DbError>{
        let stats = self.stat_service.get_stats(iter).await?;
        Ok(stats)
    }

    pub async fn get_ape(&self, iter: &Iteration) -> Result<Vec<APE>, DbError>{
        let ape = self.repo.get_ape(iter).await?;
        Ok(ape)
    }

    pub async fn get_rpe(&self, iter: &Iteration) -> Result<Vec<RPE>, DbError>{
        let rpe = self.repo.get_rpe(iter).await?;
        Ok(rpe)
    }

    async fn get_dataset_name(&self, iter: &Iteration) -> Result<String, RunError>{

        let ds = self.repo.get_dataset(iter).await.unwrap();

        Ok(ds.name)
    }

    async fn get_parents_string(&self, iter: &Iteration) -> Result<String, RunError>{

        let te = self.repo.get_test_execution_thing(iter).await.unwrap();
        let ar = self.repo.get_algorithm_run_thing(iter).await.unwrap();

        let iteration_id = iter.id.clone()
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();

        let te_str = te.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
        let ar_str = ar.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
        let it_str = iteration_id.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();


        Ok(format!("{te_str}/{ar_str}/{it_str}"))
    }

    async fn start_container(&self, iteration: &Iteration) -> Result<(), DbError> {

        let options = Some(container::CreateContainerOptions{
            name: &iteration.container.container_name,
            platform: None,
        });

        let dataset = self.repo.get_dataset(iteration).await?; //get the dataset
        let algorithm: Algorithm = self.repo.get_algorithm(iteration).await?; //get the algorithm

        let target_id: Option<Thing> = algorithm.current_params;

        let current_slam_config = self.params_service.repo.get_by_id(target_id).await?.unwrap();

        let mut rng = rand::rng();
        let random_seed = rng.next_u32();

        let mut file_name = String::from("/tmp/");
        let mut temp_file_name = format!("/tmp/temp_params_{}.yaml", random_seed);

        current_slam_config.save_to_file(&temp_file_name);

        // get test definition, if it matches the Drop test need to mount an 
        // additional file to /rustle/config/drop_config.yaml!

        //Set up the binds config to mount these volumes inside our container
        let mut hm: HashMap<&str, &String> = HashMap::from([
            ("/rustle/dataset/", &dataset.dataset_path)
        ]);

        let mut drop_file = String::new();
        let mut cut_file = String::new();

        //Add an extra YAML file for the Drop configurations.
        match &iteration.test_type{
            crate::models::TestType::Simple => {
                hm.insert("/rustle/config/params.yaml", &temp_file_name);
            },
            crate::models::TestType::Speed(_) => {
                hm.insert("/rustle/config/params.yaml", &temp_file_name);
            },
            crate::models::TestType::Drop(drop_params) => {
                //Add to the cache!
                if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
                    let cache_dir = proj_dirs.cache_dir();
                    let cache_dir_str = cache_dir.to_str().ok_or(RunError::Execution("Unable to find application path".to_owned())).unwrap();

                    let file_name = format!("{}_drop.yaml",&iteration.container.container_name);

                    let full_path = format!("{}/{}", cache_dir_str, file_name);
                    drop_file = full_path;

                    current_slam_config.save_to_file(&format!("{}/{}", cache_dir_str, file_name));

                    hm.insert("/rustle/config/params.yaml", &drop_file);

                }
            },
            crate::models::TestType::Cut(cut_params) => {
                //Add to the cache!
                if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
                    let cache_dir = proj_dirs.cache_dir();
                    let cache_dir_str = cache_dir.to_str().ok_or(RunError::Execution("Unable to find application path".to_owned())).unwrap();

                    let file_name = format!("{}_cut.yaml",&iteration.container.container_name);

                    let full_path = format!("{}/{}", cache_dir_str, file_name);
                    cut_file = full_path;

                    current_slam_config.save_to_file(&format!("{}/{}", cache_dir_str, file_name));

                    hm.insert("/rustle/config/params.yaml", &cut_file);

                }
            }
        };

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
        

        // Create a PortBinding mapping container port to host port
        let port_bindings = Some(HashMap::from([
            ("57331/tcp".to_string(), Some(vec![
                PortBinding {
                    host_ip: Some("0.0.0.0".to_string()), // listen on all interfaces
                    host_port: Some("57331".to_string()), // host port
                }
            ]))
        ]));

        //Setup container flags (cmd to execute, env variables, volumes to mount, etc...). 
        let config_docker = container::Config {
            image: Some(iteration.container.image_name.clone()),
            cmd: Some(vec!["roscore".to_string()]),
            exposed_ports: Some(HashMap::from([
                ("57331/tcp".to_string(), HashMap::new())
            ])),
            host_config: Some(HostConfig {
                binds: Some(path_mounts),
                ulimits: Some(vec![ResourcesUlimits{name:Some("nofile".to_string()),soft:Some(1024), hard:Some(524288)}]),
                port_bindings,
                ..Default::default()
            }),
            ..Default::default()
        };

        //Get the docker socket somehow
        self.docker.create_container(options, config_docker).await.unwrap();


        let _ = self.docker.start_container::<String>(&iteration.container.container_name, None).await;

        Ok(())
    }


    async fn record_task_ws(topic: &str, iteration_id: &Thing, odom_repo: OdometryRepo){

        let ten_sec = time::Duration::from_secs(5);
        thread::sleep(ten_sec);

        let (mut ws, _) = connect_async("ws://localhost:57331").await.unwrap();
        let topic_type = Self::resolve_topic_type(&mut ws, topic, 10).await.unwrap();

        // Subscribe to a topic
        let msg = json!({
            "op": "subscribe",
            "topic": topic,
            "type": topic_type
        });
        ws.send(tungstenite::Message::Text(msg.to_string().into())).await.unwrap();

        // Receive messages
        while let Some(Ok(msg)) = ws.next().await {
            if let tungstenite::Message::Text(data) = msg {

                match Self::convert_to_ros_msg(data.to_string()){
                    Ok(r) => {

                        let odometry_repo = odom_repo.clone();
                        let it_thing = iteration_id.clone();

                        // Spawn background task (does not block the loop)
                        tokio::spawn(async move {
                            if let Err(e) = Self::process_message(&odometry_repo, r, &it_thing).await {
                                warn!("Error processing messages: {e}");
                            }
                        });            

                    }
                    Err(e) => {
                        warn!("{e:}");
                    }
                }
            }
        }

    }








    // Call rosapi/topic_type repeatedly until we get a non-empty type.
    async fn resolve_topic_type(
        ws: &mut tokio_tungstenite::WebSocketStream<MaybeTlsStream<TcpStream>>,
        topic: &str,
        max_attempts: usize,
    ) -> Result<String, RosError> {
        for attempt in 0..max_attempts {
            // generate a unique id for this request
            let millis = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis();
            let id = format!("topic_type_{}_{}", millis, attempt);

            let req = json!({
                "op": "call_service",
                "service": "/rosapi/topic_type",
                "args": { "topic": topic },
                "id": id
            });

            let txt = req.to_string();
            warn!("Sending topic_type request (attempt {}): {}", attempt + 1, txt);
            ws.send(tungstenite::Message::Text(txt.into())).await.unwrap();

            // wait for response matching our id; allow multiple incoming messages and short timeouts
            let mut got_type: Option<String> = None;
            let deadline = Duration::from_secs(1); // wait up to 2s per attempt

            // Loop reading messages until timeout or we find the response for our id
            loop {
                match timeout(deadline, ws.next()).await {
                    Ok(Some(Ok(tungstenite::Message::Text(data)))) => {

                        let parsed: serde_json::Value = serde_json::from_str(&data).unwrap_or_default();

                        // response may be: { "op":"service_response", "id": "<id>", "values": {"type": "..."}, ... }
                        if parsed.get("id") == Some(&json!(id)) {
                            let t = parsed
                                .get("values")
                                .and_then(|v| v.get("type"))
                                .and_then(|v| v.as_str())
                                .unwrap_or("")
                                .to_string();

                            if !t.is_empty() {
                                got_type = Some(t);
                                break;
                            } else {
                                warn!("Rosapi returned empty type for {}, will retry", topic);
                                break; // break inner loop and retry
                            }
                        } else {
                            // not our response; continue reading
                            debug!("Not our service response (id mismatch).");
                            continue;
                        }
                    }

                    Ok(Some(Ok(tungstenite::Message::Ping(_)))) => {
                        debug!("Ping (while waiting)");
                        // respond automatically, tungstenite may handle pong automatically, but you can reply if needed
                    }

                    Ok(Some(Ok(other_msg))) => {
                        debug!("Other message while resolving type: {:?}", other_msg);
                        // continue reading for our id
                    }

                    Ok(Some(Err(e))) => {
                        return Err(RosError::Query("Unable to find ROS topic type".to_owned()));
                    }

                    Ok(None) => {
                        return Err(RosError::Query("Unable to find ROS topic type".to_owned()));
                    }

                    Err(_) => {
                        // timeout waiting for messages matching our id — break to retry
                        debug!("Timed out waiting for service response (attempt {})", attempt + 1);
                        break;
                    }
                }
            }

            if let Some(t) = got_type {
                return Ok(t);
            }

            // small delay before retrying
            sleep(Duration::from_millis(500)).await;
        }

        Err(RosError::Query(format!(
            "Could not resolve type for topic '{}' after {} attempts",
            topic,
            max_attempts
        )))
    }








    fn convert_to_ros_msg(data: String) -> Result<RosMsg, RosError> {
        // Parse JSON
        let v: Value = serde_json::from_str(&data)
            .map_err(|_| RosError::FormatError(data.clone()))?;

        let inner_msg = v.get("msg")
            .ok_or_else(|| RosError::FormatError(format!("Missing 'msg' field: {}", data)))?;

        // Determine top-level keys
        let top_fields: Vec<&str> = inner_msg.as_object()
            .ok_or_else(|| RosError::FormatError(format!("Expected object in 'msg': {}", inner_msg)))?
            .keys()
            .map(|s| s.as_str())
            .collect();

        // Create RosMsg
        let ros = RosMsg::new(top_fields)?;
        let ros = ros.from_json(inner_msg)?;

        Ok(ros)
    }

    async fn process_message(
        odom_repo: &OdometryRepo,
        msg: RosMsg,
        iteration_id: &Thing
    ) -> Result<(), ProcessingError> {
        let odom = msg.as_odometry().unwrap();
        let mut db_odom = Odometry::new(odom.header);
        
        // Copy relevant fields
        db_odom.child_frame_id = odom.child_frame_id;
        db_odom.pose = odom.pose;
        db_odom.twist = odom.twist;

        odom_repo.save(&mut db_odom, iteration_id).await?;
        Ok(())
    }

    async fn remove_container(&self, container_name:&str){
        //Remove the containers

        let _ = self.docker.remove_container(
            container_name,
            Some(
                RemoveContainerOptions{
                    force: true,
                    ..Default::default()
                }
            )
        ).await;
    }

    async fn compute_metrics<R: EvoArg>(&self, iter: &Iteration, args: &R, result_path: &String, dataset_path: &String) -> Result<StatisticalMetrics, EvoError>{

        //let ground_truth_data = self.repo.get_dataset(iter)
        //    .await.unwrap()
        //    .ground_truth  // Clone the Option first
        //    .ok_or_else(|| ProcessingError::NotFound("Dataset Odometries were not found".into())).unwrap();
        
        //let _ = Self::write_file(&ground_truth_data, "groundtruth", &mut PathBuf::from_str(&dataset_path).unwrap());

        let odoms: Vec<Odometry> = self.repo.get_odometries(iter).await.unwrap();
        let _ = Self::write_file(&odoms, &iter.container.container_name, &mut PathBuf::from_str(&result_path).unwrap());
        let _ = Self::write_file(&odoms, &iter.container.container_name, &mut PathBuf::from_str(&result_path).unwrap());

        let evo_ape_str = args.compute(&format!("{dataset_path}/groundtruth"), &format!("{result_path}/{}",&iter.container.container_name))?;
        
        let _ = run_metrics_py(&format!("{dataset_path}/groundtruth"), &format!("{result_path}/{}",&iter.container.container_name), &self.config, &result_path);

        let metric = StatisticalMetrics::from_str(&evo_ape_str); //TODO Dont unwrap() this
        metric

    }

    fn write_file<'a>(data: &[Odometry], name: &str, path: &mut PathBuf) -> Result<(), std::io::Error>{

        path.push(name.replace("/", "_"));

        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)?;
        
            for odom in data {
                writeln!(file, "{}", odom)?;
            };
        Ok(())
    }


}

pub fn parse_rosbag_line(line: &str, iter_num: u64, algo: String) -> Option<ProgressMessage> {
    let re = regex::Regex::new(r"Bag Time: (\d+\.\d+)\s+Duration: (\d+\.\d+) / (\d+\.\d+)").ok()?;
    let caps = re.captures(line)?;
    Some(ProgressMessage {
        iteration_num: iter_num,
        bag_time: caps.get(1)?.as_str().parse().ok()?,
        duration: caps.get(2)?.as_str().parse().ok()?,
        total_duration: caps.get(3)?.as_str().parse().ok()?,
        algo,
    })
}
