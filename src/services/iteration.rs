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
use crate::models::metrics::metric::StatisticalMetrics;

use crate::models::metrics::pose_error::PoseErrorMetrics;
use crate::utils::evo_wrapper::{EvoApeArg, EvoRpeArg};
use crate::{
    db::{iteration::IterationRepo, OdometryRepo}, 
    models::{iteration::{DockerContainer, Iteration}, 
    ros::ros_msg::RosMsg, Algorithm, ContainerStats, Dataset, Odometry}, 
    services::error::{DbError, ProcessingError, RosError}, 
    utils::evo_wrapper::EvoArg
};

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

        //let cmd = format!("rosbag play -r {} --clock /rustle/dataset/*.bag", algorithm_run.bag_speed);
        let cmd = format!("rosbag play -d 9 -r {} --clock -u 50 /rustle/dataset/*.bag", algorithm_run.bag_speed);
        let rustle_cmd = format!("roslaunch rustle rustle.launch --wait test_type:={}", &iter.test_type);

        

        

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

        let ten_sec = time::Duration::from_secs(5);
        thread::sleep(ten_sec);

        let _ = self.remove_container(&iter.container.container_name).await;

        let ape_args = EvoApeArg{
                    //plot: Some(PlotArg::default()),
                    plot: None,
                    ..Default::default()
                };

        let rpe_args = EvoRpeArg{
            //plot: Some(PlotArg::default()),
            plot: None,
            ..Default::default()
        };

        let ape = self.compute_metrics(&iter, &ape_args, None).await.unwrap();
        let rpe = self.compute_metrics(&iter, &rpe_args, None).await.unwrap();

        //Create a metric object.
        //let metric = Metr

        let iteration_id_clone = iter.id
            .clone()  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();




        let metric = self.metric_service.create_pose_error_metric(iteration_id_clone, ape, rpe).await.unwrap();


        Ok(())
    }

    async fn start_container(&self, iteration: &Iteration) -> Result<(), DbError> {

        let options = Some(container::CreateContainerOptions{
            name: &iteration.container.container_name,
            platform: None,
        });

        let dataset = self.repo.get_dataset(iteration).await?; //get the dataset
        let algorithm = self.repo.get_algorithm(iteration).await?; //get the algorithm

        //Set up the binds config to mount these volumes inside our container
        let hm: HashMap<&str, &String> = HashMap::from([
            ("/rustle/dataset/", &dataset.dataset_path),
            ("/rustle/config/params.yaml", &algorithm.parameters),
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


    async fn compute_metrics<R: EvoArg>(&self, iter: &Iteration, args: &R, output_path: Option<&str>) -> Result<StatisticalMetrics, EvoError>{

        //Write TUM files to temporary directory
        let mut path = match output_path{
            Some(p) => {
                PathBuf::from(p)
            },
            None => {
                env::temp_dir()
            }
        };

        //WRITE GT TO A FILE -> THIS SHOULD NOT BE NEEDED IF THERE IS ALREADY A GT FILE. TODO
        let mut path_gt = path.clone();

        let ground_truth_data = self.repo.get_dataset(iter)
            .await.unwrap()
            .ground_truth  // Clone the Option first
            .ok_or_else(|| ProcessingError::NotFound("Dataset Odometries were not found".into())).unwrap();       

        
        Self::write_file(&ground_truth_data, "groundtruth", &mut path_gt);


        let odoms: Vec<Odometry> = self.repo.get_odometries(iter).await.unwrap();
        Self::write_file(&odoms, &iter.container.container_name, &mut path);

        let evo_ape_str = args.compute(path_gt.to_str().unwrap(), path.to_str().unwrap())?;
        
        let metric = StatisticalMetrics::from_str(&evo_ape_str); //TODO Dont unwrap() this
        metric

    }

    fn write_file<'a>(data: &[Odometry], name: &str, path: &mut PathBuf) -> Result<(), std::io::Error>{

        path.push(name.replace("/", "_"));

        let mut file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;
        
            for odom in data {
                writeln!(file, "{}", odom)?;
            };
        Ok(())
    }


}