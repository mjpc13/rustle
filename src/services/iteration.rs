use std::fs;
use std::path::Path;
use std::str::FromStr;
use std::{collections::HashMap, env, fs::OpenOptions, path::PathBuf, sync::Arc, thread, time};
use std::io::Write;

use bollard::{container::{self, RemoveContainerOptions, StatsOptions}, exec::{CreateExecOptions, StartExecResults}, secret::{HostConfig, ResourcesUlimits}, Docker};
use chrono::Utc;
use clap::error;
use futures_util::{future, StreamExt};
use log::{debug, info, trace, warn};
use rand::{distr::Alphanumeric, Rng};
use tokio::{select, sync::Mutex};

use surrealdb::sql::Thing;
use tokio_util::sync::CancellationToken;
use yaml_rust2::YamlLoader;

use crate::models::metric::Metric;
use crate::models::metrics::memory::MemoryMetrics;
use crate::models::metrics::metric::StatisticalMetrics;

use crate::models::metrics::pose_error::PoseErrorMetrics;
use crate::models::metrics::{ContainerStats, CpuMetrics};
use crate::utils::evo_wrapper::{EvoApeArg, EvoRpeArg, PlotArg};
use crate::{
    db::{iteration::IterationRepo, OdometryRepo}, 
    models::{iteration::{DockerContainer, Iteration}, 
    ros::ros_msg::RosMsg, Algorithm, Dataset, Odometry}, 
    services::error::{DbError, ProcessingError, RosError}, 
    utils::evo_wrapper::EvoArg
};

use directories::{BaseDirs, UserDirs, ProjectDirs};

use super::error::EvoError;
use super::{MetricService};
use super::{dataset, error::RunError, DatasetService, RosService, StatService};
#[derive(Clone)]
pub struct IterationService {
    repo: IterationRepo,
    ros_service: RosService,
    dataset_service: DatasetService,
    stat_service: StatService,
    metric_service: MetricService,
    docker: Arc<Docker>,  // Assuming you have Docker client setup
}

impl IterationService {
    pub fn new(repo: IterationRepo, docker: Arc<Docker>, ros_service: RosService, dataset_service: DatasetService, stat_service: StatService, metric_service: MetricService,) -> Self {
        Self { repo, docker, ros_service, dataset_service, stat_service, metric_service }
    }

    pub async fn create(&self, iter_number: u8, algo:Algorithm, algorithm_run_id: &Thing, test_type: String) -> Result<(), DbError> {
        
        let sanitized = algo.name.replace(|c: char| !c.is_alphanumeric(), "_")
        .to_lowercase();

        let rng = rand::thread_rng();
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
        };

        let _ = self.repo.save(&mut iteration, algorithm_run_id).await;
        Ok(())
    }

    pub async fn run(&self, iter: Iteration) -> Result<(), RunError> {

        let algorithm = self.repo.get_algorithm(&iter).await.unwrap(); //Maybe wrap in an Arc<>, Also, maybe wrap iter in an Arc
    
        // -- CREATE THE CONTAINER ---
        let _ = self.start_container(&iter).await;

        //get the corresponding algorithm run
        let algorithm_run = self.repo.get_algorithm_run(&iter).await.unwrap();

        let cmd = format!("rosbag play -r {} --clock /rustle/dataset/*.bag", algorithm_run.bag_speed);
        //let cmd = format!("rosbag play -d 9 -r {} --clock -u 50 /rustle/dataset/*.bag", algorithm_run.bag_speed);
        let rustle_cmd = format!("roslaunch rustle-ros rustle.launch --wait test_type:={}", &iter.test_type);

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
        let ten_sec = time::Duration::from_secs(10);
        thread::sleep(ten_sec);

        //let config_clone = self.config.clone();
        let roslaunch_id = execs[0].id.clone();
        let task_token = token.clone();
        let docker_clone = self.docker.clone();
        let iter_clone = iter.clone();

        //Execute the rustle.launch
        let roslaunch_task = tokio::spawn(async move {
            if let StartExecResults::Attached { mut output, mut input } = docker_clone.start_exec(&roslaunch_id, None).await.unwrap() {
                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {
                           //trace!("ROS MSG: {msg}");
                        },
                        _ = task_token.cancelled()=>{
                            info!("Container Stopped");
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

        //Only add the groundtruth if there is no GT in the Dataset Object
        let dataset = self.repo.get_dataset(&iter).await.unwrap(); //get the dataset

        let is_gt_empty = match dataset.ground_truth{
            Some(_) => false,
            None => {
                true
            },
        };

        let topic_list = match is_gt_empty{
            true => {
                let topic = dataset.ground_truth_topic.clone()  // Clone the Option<String> first
                    .ok_or_else(|| ProcessingError::MissingField("ground_truth_topic".into())).unwrap();
                let mut topics = algorithm.odom_topics;
                topics.push(topic);
                topics
            },
            false => algorithm.odom_topics
        };
        let gt_topic = dataset.ground_truth_topic.clone()  // Clone the Option<String> first
         .ok_or_else(|| ProcessingError::MissingField("ground_truth_topic".into())).unwrap();
        
        //TODO: OPTIMIZE THIS, THINK OF A BETTER WAY TO SAVE THE GT TOPIC
        let res_tasks: Vec<_> = topic_list
            .into_iter()
            .map( |s: String| {
                let iter_id_clone = iter.id
                .clone()  // Clone the Option first
                .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();

                let token_clone = token.clone();
                let container_name = iter.container.container_name.clone();
                let docker_clone = self.docker.clone();
                let ros_service_clone = self.ros_service.clone();
                let dataset_service_clone = self.dataset_service.clone();
                let dataset_clone = dataset.clone();
                let gt_topic_clone = gt_topic.clone();
                
                tokio::spawn(async move {
                    // LOGIC TO SAVE ODOMS TO DB 
                    if s.eq(&gt_topic_clone){
                        Self::record_ground_truth(dataset_service_clone, 
                          docker_clone, 
                          container_name, 
                          &s,
                          token_clone, &dataset_clone).await;
                    }else{
                        let _ = Self::record_task(ros_service_clone, docker_clone, container_name, &s, token_clone, iter_id_clone).await;
                    }
                })

            })
            .collect();


       // Wait a certain number of seconds for the algorithm to init
       let ten_sec = time::Duration::from_secs(5);
       thread::sleep(ten_sec);

       //Start the STATS collection
        let task_id_clone = iter.container.container_name.clone();
        let iteration_id_clone = iter.id
            .clone()  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();       
        let stream_token = token.clone();

        let docker_clone = self.docker.clone();
        let stat_service_clone = self.stat_service.clone();

       let stream_task = tokio::spawn(async move{
           let stream= &mut docker_clone
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
                while let Some(Ok(msg)) = output.next().await {
                    //trace!("ROSBAG: {msg}");
                }
            } else {
                warn!("STREAM PLAY ENDED");
                unreachable!();
            }
                //Loop to give the prints;
        });
                
        let _ = tokio::join!(rosplay_task);
        debug!("Stopped rosbag play, send cancel signal to the other tasks");
        token.cancel(); // the end of the rosbag will be the first point where the other tasks need
        // to stop
        let _ = tokio::join!(roslaunch_task);
        debug!("Stopped roslaunch");

        let ten_sec = time::Duration::from_secs(1);
        thread::sleep(ten_sec);

        let iteration_id_clone = iter.id
            .clone()  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();

        //CLEAN RESIDUAL CONTAINERS
        let _ = self.remove_container(&iter.container.container_name).await;


        //Compute the frequency
        let freq = self.repo.get_odom_frequency(&iter).await.unwrap();
        let freq_metric = StatisticalMetrics::from_single_value(freq);

        let _ = self.metric_service.create_freq_metric(iteration_id_clone.clone(), freq_metric).await; // add to DB


        let stats = self.stat_service.get_stats(&iter).await.unwrap();

        //Compute CPU stats
        let cpu_metric_opt = CpuMetrics::from_stats(&stats);
        if let Some(cpu_metric) = cpu_metric_opt {
            let _ = self.metric_service.create_cpu_metric(iteration_id_clone.clone(), cpu_metric).await.unwrap();  
        }

        //Compute Memory stats
        let memory_metric_opt = MemoryMetrics::from_stats(&stats).unwrap();
        if let Some(memory_metric) = memory_metric_opt {
            let _ = self.metric_service.create_memory_metric(iteration_id_clone.clone(), memory_metric).await.unwrap();  
        }

        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned()))?;


            //Create the folder for this iteration:
            // test_execution_id/algo_run_id/iteration_id/
            //something like self.repo.get_parents_id()
            let iter_path = self.get_parents_string(&iter).await?;
            let dataset_path = self.get_dataset_string(&iter).await?;

            let full_path = format!("{data_dir_str}/{iter_path}");
            let full_dataset_path = format!("{data_dir_str}/{dataset_path}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();
            fs::create_dir_all(&full_dataset_path).unwrap();


            

            //Compute the APE and RPE metrics
            let ape_args = EvoApeArg{
                //plot: Some(PlotArg::default()),
                plot: Some(PlotArg{
                    path: full_path.clone(),
                    ..Default::default()
                }),
                ..Default::default()
            };

            let rpe_args = EvoRpeArg{
                plot: Some(PlotArg{
                    path: full_path.clone(),
                    ..Default::default()
                }),
                ..Default::default()
            };

            let ape = self.compute_metrics(&iter, &ape_args, &full_path, &full_dataset_path).await.unwrap();
            let rpe = self.compute_metrics(&iter, &rpe_args, &full_path, &full_dataset_path).await.unwrap();

            let metric = self.metric_service.create_pose_error_metric(iteration_id_clone, ape, rpe).await.unwrap();


        };

        Ok(())
    }

    async fn get_dataset_string(&self, iter: &Iteration) -> Result<String, RunError>{

        let ds = self.repo.get_dataset_thing(iter).await.unwrap();
        let ds_str = ds.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();

        Ok(format!("{ds_str}"))
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
        let algorithm = self.repo.get_algorithm(iteration).await?; //get the algorithm

        // get test definition, if it matches the Drop test need to mount an 
        // additional file to /rustle/config/drop_config.yaml!
        let test_def = self.repo.get_test_def(iteration).await?;


        //Set up the binds config to mount these volumes inside our container
        let mut hm: HashMap<&str, &String> = HashMap::from([
            ("/rustle/dataset/", &dataset.dataset_path)
        ]);

        let mut drop_file = String::new();
        let mut cut_file = String::new();

        //Add an extra YAML file for the Drop configurations.
        match test_def.test_type{
            crate::models::TestType::Simple => {
                hm.insert("/rustle/config/params.yaml", &algorithm.parameters);
            },
            crate::models::TestType::Speed(speed_test_params) => {
                hm.insert("/rustle/config/params.yaml", &algorithm.parameters);
            },
            crate::models::TestType::Drop(drop_params) => {
                //Add to the cache!
                if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
                    let cache_dir = proj_dirs.cache_dir();
                    let cache_dir_str = cache_dir.to_str().ok_or(RunError::Execution("Unable to find application path".to_owned())).unwrap();

                    let mut params_string = fs::read_to_string(&algorithm.parameters).unwrap();

                    let drop_yaml = drop_params.update_file(&params_string);

                    let file_name = format!("{}_drop.yaml",&iteration.container.container_name);

                    let full_path = format!("{}/{}", cache_dir_str, file_name);
                    drop_file = full_path;

                    let _ = fs::write(format!("{}/{}", cache_dir_str, file_name), drop_yaml);

                    hm.insert("/rustle/config/params.yaml", &drop_file);

                }
            },
            crate::models::TestType::Cut(cut_params) => {
                //Add to the cache!
                if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
                    let cache_dir = proj_dirs.cache_dir();
                    let cache_dir_str = cache_dir.to_str().ok_or(RunError::Execution("Unable to find application path".to_owned())).unwrap();

                    let mut params_string = fs::read_to_string(&algorithm.parameters).unwrap();

                    let cut_yaml = cut_params.update_file(&params_string);

                    let file_name = format!("{}_cut.yaml",&iteration.container.container_name);

                    let full_path = format!("{}/{}", cache_dir_str, file_name);
                    cut_file = full_path;

                    let _ = fs::write(format!("{}/{}", cache_dir_str, file_name), cut_yaml);

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
        

        //Setup container flags (cmd to execute, env variables, volumes to mount, etc...).
        let config_docker = container::Config {
            image: Some(iteration.container.image_name.clone()),
            cmd: Some(vec!["roscore".to_string()]),
            host_config: Some(HostConfig {
                binds: Some(path_mounts),
                ulimits: Some(vec![ResourcesUlimits{name:Some("nofile".to_string()),soft:Some(1024), hard:Some(524288)}]),
                ..Default::default()
            }),
            ..Default::default()
        };

        //Get the docker socket somehow
        self.docker.create_container(options, config_docker).await.unwrap();


        let _ = self.docker.start_container::<String>(&iteration.container.container_name, None).await;

        Ok(())
    }

    async fn record_task(ros_service: RosService, docker: Arc<Docker>, container_id: String, topic: &str, cancelation_token: Arc<CancellationToken>, iteration_id: Thing){

        let cmd = format!("rostopic echo {topic}");

        // Check how I'm gonna record the localization
        let record_options_future = docker
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

        if let StartExecResults::Attached { mut output, .. } = docker.start_exec(&record_exec_id, None).await.unwrap() {

                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {

                            match Self::convert_to_ros(msg.to_string()){
                                Ok(r) => {
                                    //let odom = r.as_odometry().unwrap();
                                    let _ = ros_service.process_message(r, &iteration_id).await;
                                }
                                Err(e) => {
                                    info!("{e:}");
                                }
                            }

                        },
                        _ = cancelation_token.cancelled()=>{
                            
                            let record_options_future = docker
                                .create_exec(
                                &container_id,
                                CreateExecOptions {
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["/bin/bash", "-l", "-c", "rosnode", "kill", "-a", "2>/dev/null"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

                            docker.start_exec(&record_options_future.id, None).await.unwrap();

                            let ten_sec = time::Duration::from_secs(5);
                            thread::sleep(ten_sec);

                            break;
                        }
                    }
                }

            }


    }


    async fn record_ground_truth(dataset_service: DatasetService, docker: Arc<Docker>, container_id: String, topic: &str, cancelation_token: Arc<CancellationToken>, dataset: &Dataset){

        let cmd = format!("rostopic echo {topic}");

        // Check how I'm gonna record the localization
        let record_options_future = docker
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


        if let StartExecResults::Attached { mut output, .. } = docker.start_exec(&record_exec_id, None).await.unwrap() {

                loop{
                    select!{
                        Some(Ok(msg)) = output.next() => {

                            match Self::convert_to_ros(msg.to_string()){
                                Ok(r) => {

                                    let odom = r.as_odometry().unwrap();
                                    let mut db_odom = Odometry::new(odom.header);
                                    
                                    // Copy relevant fields
                                    db_odom.child_frame_id = odom.child_frame_id;
                                    db_odom.pose = odom.pose;
                                    db_odom.twist = odom.twist;

                                    //let odom = r.as_odometry().unwrap();
                                    //WRITE CODE TO ADD TO GROUNDTRUTH
                                    let _ = dataset_service.add_ground_truth(&dataset, db_odom).await;
                                    //let _ = dataset_service.add_ground_truth
                                }
                                Err(e) => {
                                    warn!("{e:}");
                                }
                            }

                        },
                        _ = cancelation_token.cancelled()=>{
                            
                            let record_options_future = docker
                                .create_exec(
                                &container_id,
                                CreateExecOptions {
                                    attach_stderr: Some(true),
                                    cmd: Some(vec!["/bin/bash", "-l", "-c", "rosnode", "kill", "-a", "2>/dev/null"]),
                                    ..Default::default()
                                },
                            ).await.unwrap();

                            docker.start_exec(&record_options_future.id, None).await.unwrap();

                            let ten_sec = time::Duration::from_secs(5);
                            thread::sleep(ten_sec);

                            break;
                        }
                    }
                }

            }

    }


    fn convert_to_ros(msg: String) -> Result<RosMsg, RosError> {

        let yaml = match YamlLoader::load_from_str(&msg.replace("\n---\n", "")){
            Ok(y) => y,
            Err(e) => {
                return Err(RosError::FormatError(msg.into()))
            },
        };
        
        let ros_msg = yaml[0].as_hash().ok_or_else(|| RosError::FormatError(format!("YAML msg: {:?}", yaml[0])))?;



        let top_fields: Vec<_> = ros_msg
            .keys()
            .into_iter()
            .map(|y|{
                y.as_str().unwrap()
            }).collect();

        let ros = RosMsg::new(top_fields)?;

        return ros.from_yaml(yaml[0].clone());
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

        //WRITE GT TO A FILE -> THIS SHOULD NOT BE NEEDED IF THERE IS ALREADY A GT FILE. TODO

        let ground_truth_data = self.repo.get_dataset(iter)
            .await.unwrap()
            .ground_truth  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Dataset Odometries were not found".into())).unwrap();
        
        Self::write_file(&ground_truth_data, "groundtruth", &mut PathBuf::from_str(&dataset_path).unwrap());

        let odoms: Vec<Odometry> = self.repo.get_odometries(iter).await.unwrap();
        Self::write_file(&odoms, &iter.container.container_name, &mut PathBuf::from_str(&result_path).unwrap());

        let evo_ape_str = args.compute(&format!("{dataset_path}/groundtruth"), &format!("{result_path}/{}",&iter.container.container_name))?;
        
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