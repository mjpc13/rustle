use std::error::Error;
use std::{fs::File, sync::Arc};

use bollard::{image::CreateImageOptions, Docker};
use bollard::errors::Error as DockerError;
use log::{info, trace, warn};

use crate::db::TuningRepo;
use crate::models::metrics::memory::MemoryMetrics;
use crate::models::slam_config::SLAMConfig;
use crate::models::tuning::tuning_config::{self, TuningConfig, format_hashmap};
use crate::models::tuning::simulated_annealing::{cost_function, gaussian_perturbation, integer_perturbation};
use crate::models::tuning::{SimulatedAnnealingConfig, TuneType};
use crate::models::{Iteration, Pose, TestDefinition, TestExecution, TestType};
use crate::services::params::ParamsService;
use crate::services::{AlgorithmRunService, AlgorithmService, DatasetService, DbError, IterationService, MetricService, TestExecutionService};
use crate::{models::Algorithm, db::params::ParamsRepo, services::error::{ValidationError}};
use futures_util::stream::{StreamExt};
use super::error::ProcessingError;

use crate::services::error::*;

use chrono::Utc;

use tokio::sync::mpsc;
use crate::models::ProgressMessage;
use indicatif::{MultiProgress, ProgressBar, ProgressStyle, ProgressDrawTarget};

use std::sync::Mutex;
use std::collections::HashMap;
use tokio::task;

use crate::models::metric::MetricType::{self, Cpu, Frequency, Memory, PoseError};
use crate::models::metrics::{Metric, PoseErrorMetrics};

use std::io::{BufReader, Write, BufRead};

use serde_json::{Value, json};
use std::fs;

use rand::Rng;
use csv::Writer;

use std::time::{Duration, Instant};


pub struct TuningService {
    pub repo: TuningRepo,
    iteration_service: IterationService,
    algo_run_service: AlgorithmRunService,
    test_exec_service: TestExecutionService,
    dataset_service: DatasetService,
    algo_service: AlgorithmService,
    params_service: ParamsService,
    metrics_service: MetricService,
    docker: Arc<Docker>,
}

impl TuningService {
    pub fn new(repo: TuningRepo, iteration_service: IterationService, algo_run_service: AlgorithmRunService, test_exec_service: TestExecutionService, dataset_service: DatasetService, algo_service: AlgorithmService, params_service: ParamsService, metrics_service: MetricService, docker: Arc<Docker>) -> Self {
        Self { repo, 
               iteration_service, 
               algo_run_service, 
               test_exec_service, 
               dataset_service, 
               algo_service, 
               params_service, 
               metrics_service, 
               docker }
    }

    pub async fn create_from_yaml(&self, file_name: &str) -> Result<TuningConfig, Box<dyn std::error::Error>> {
        let mut tuning_config = TuningConfig::new();

        self.repo.save(&mut tuning_config).await?;

        Ok(tuning_config)
    }

    pub async fn save_to_db(&self, config: &mut TuningConfig) -> Result<(), DbError> {
        self.repo.save(config).await?;
        Ok(())
    }

    pub async fn get_all(&self) -> Result<Vec<TuningConfig>, DbError> {
        let results = self.repo.get_all().await?;
        Ok(results)
    }

    pub async fn get_by_name(&self, name: String) -> Result<Option<TuningConfig>, DbError> {
        match self.repo.get_by_name(name).await {
            Ok(cfg) => {
                Ok(Some(cfg))
            }
            Err(_) => {
                Err(DbError::NotFound(String::from("Tuning config not found in the database.")))
            }
        }
    }

    pub async fn run_tuning_algo(&mut self, tuning_test: &mut TuningConfig) -> Result<(), Box<dyn std::error::Error>> {
        match tuning_test.tuning_type.clone().unwrap() {
            TuneType::GridSearch(_) => {
                self.run_grid_search(tuning_test).await?;
            }
            TuneType::RandomSearch(_) => {
                self.run_random_search(tuning_test).await?;
            }
            TuneType::SimulatedAnnealing(_) => {
                self.run_simulated_annealing(tuning_test, tuning_test.tuning_type.clone().unwrap().as_simulated_annealing_as_mut().unwrap()).await?;
            }
        }
        Ok(())
    }

    async fn run_grid_search(&mut self, tuning_test: &TuningConfig) -> Result<(), Box<dyn std::error::Error>> {

        if let Some(start) = tuning_test.dataset_settings.0 {
            self.iteration_service.config.rustle.dataset_start = start;
        }

        if let Some(duration) = tuning_test.dataset_settings.1 {
            self.iteration_service.config.rustle.dataset_duration = duration;
        }

        let mut total_iterations: usize = 0;
        if let Some(TuneType::GridSearch(gs_config)) = &tuning_test.tuning_type {
            total_iterations = (gs_config.get_param_array_size_by_index(None).unwrap() as usize);
        }

        let mut test_def = TestDefinition{
            id: None,
            name: String::from("dummy_test"),
            workers: 5,
            iterations: total_iterations as u64,
            dataset_name: self.dataset_service.get_dataset_name_by_id(&tuning_test.dataset_id).await?.unwrap(),
            algo_list: vec![
                self.algo_service.get_algo_name_by_id(tuning_test.algo_id.clone()).await?.unwrap(),
            ],
            test_type: TestType::Simple,
            created_at: Utc::now(),
            updated_at: Utc::now(),
        };
        let mut test_exec = TestExecution::new(test_def);
        self.test_exec_service.save_test_execution(&mut test_exec).await?;

        let algo = self.algo_service.get_by_id(tuning_test.algo_id.clone()).await?.unwrap();
        let mut algo_run = self.algo_run_service.create_run(1.0,
                                                                          total_iterations as u64,
                                                                          &test_exec.id.clone().unwrap(), 
                                                                          &algo.id.clone().unwrap(), 
                                                                          TestType::Simple).await?;

        let list_iterations = self.test_exec_service.execution_repo.get_iterations(&test_exec.id.clone().unwrap()).await?;

        let (msg_tx, mut msg_rx) = mpsc::channel::<ProgressMessage>(100);
        let multi = Arc::new(MultiProgress::new());
        let bars = Arc::new(Mutex::new(HashMap::new()));  

        tokio::spawn({
            let multi = multi.clone();
            let bars = bars.clone();
            async move {
                while let Some(msg) = msg_rx.recv().await {
                    let mut bars = bars.lock().unwrap();
                                    
                    let bar = bars.entry(msg.iteration_num).or_insert_with(|| {
                        let pb = multi.add(ProgressBar::new((msg.total_duration * 1000.0) as u64));
                        pb.set_style(
                            ProgressStyle::with_template(
                                "{elapsed_precise} | {prefix} |> {bar:40.cyan/blue} {percent}% | {pos:.2}/{len:.2} sec"
                            )
                            .unwrap()
                            .progress_chars("███████ "),
                        );
                        pb.set_prefix(format!(
                            "\x1b[93m{} | Iter {}\x1b[0m",
                            msg.algo, msg.iteration_num
                        ));
                        pb
                    });
                                    
                    bar.set_position((msg.duration * 1000.0) as u64);
                }
                                
                // Finish remaining bars
                for bar in bars.lock().unwrap().values() {
                    bar.finish();
                }
            }
        });

        let mut all_iterations_metrics: Vec<(Vec<Metric>, Vec<(String, u64)>)> = Vec::new();

        let initial_grid_point = tuning_test.generate_initial_grid_point().unwrap();
        let mut current_grid_point = initial_grid_point.clone();
        let mut current_grid_point_index = tuning_test.get_grid_point_index(&current_grid_point).unwrap();
        let initial_params = &self.params_service.get_by_id(algo.current_params.clone()).await?.unwrap().params.clone();

        let mut writer = Writer::from_path("examples/grid_search.csv")?;
        let mut csv_header: Vec<String> = Vec::new();
        let mut csv_header_keys: Vec<String> = Vec::new();

        csv_header.push(String::from("iter"));
        for (key, value) in &tuning_test.parameters_to_tune {
            csv_header.push(key.clone());
            csv_header_keys.push(key.clone());
        }
        csv_header.push(String::from("ape"));
        csv_header.push(String::from("rpe"));
        writer.write_record(&csv_header)?;

        let limit = match tuning_test.time_limit {
            Some(limit) => {
                let time_limit_seconds = (limit * (3600 as f64)) as u64;
                Duration::from_secs(time_limit_seconds)
            },
            None => Duration::from_secs(0),
        };
        let start = Instant::now();

        for i in 0..total_iterations {

            let current_params = tuning_test.get_current_params(&current_grid_point, initial_params).unwrap();
            self.params_service.update_params(algo.current_params.clone(), &current_params).await?;

            let iter_job = self.iteration_service.run(list_iterations[current_grid_point_index as usize].clone(), Some(msg_tx.clone())).await;
            match iter_job {
                Ok(_) => {
                    let iter_metrics = self.metrics_service.get_all_iteration_metrics(&list_iterations[current_grid_point_index as usize].id.clone().unwrap()).await?;
                    all_iterations_metrics.push((iter_metrics.clone(), current_grid_point.clone()));

                    let mut current_iter_metrics: (usize, f32, f32) = (0 as usize, 3.0 as f32, 2.0 as f32);
                    if let Some(current_pose_errors) = self.get_pose_error(&iter_metrics) {
                        if let (Some(iter), Some(ape), Some(rpe)) = (tuning_test.get_grid_point_index(&current_grid_point), current_pose_errors.ape.rmse, current_pose_errors.rpe.rmse) {
                            current_iter_metrics.0 = iter as usize;
                            current_iter_metrics.1 = ape;
                            current_iter_metrics.2 = rpe;
                            let mut current_iter_data: Vec<String> = Vec::new();
                            current_iter_data.push(current_iter_metrics.0.to_string());
                            for key in &csv_header_keys {
                                current_iter_data.push(current_params.get(key).unwrap().to_string());
                            }
                            current_iter_data.push(current_iter_metrics.1.to_string());
                            current_iter_data.push(current_iter_metrics.2.to_string());
                            tuning_test.save_tuning_progress(&mut writer, &current_iter_metrics, &current_iter_data)?;
                        }
                    }

                    if let Some(_) = tuning_test.time_limit {
                        if start.elapsed() >= limit {
                            println!("Time limit exceed. Saving the results and exiting...");
                            break;   
                        }
                    }
                    else if all_iterations_metrics.len() > tuning_test.early_stopping_params.unwrap().0.clone() as usize {
                        if self.should_tuning_stop(&all_iterations_metrics, &tuning_test.metrics_weights.unwrap(), &tuning_test.early_stopping_params.unwrap()) {
                            println!("Early stopping triggered");
                            break; 
                        }
                        else {
                            current_grid_point = tuning_test.generate_next_grid_point(current_grid_point).unwrap();
                            current_grid_point_index = tuning_test.get_grid_point_index(&current_grid_point).unwrap();                            
                        }
                    }
                    else {
                        current_grid_point = tuning_test.generate_next_grid_point(current_grid_point).unwrap();
                        current_grid_point_index = tuning_test.get_grid_point_index(&current_grid_point).unwrap();
                    }

                },
                Err(e) => {
                    warn!("Iteration number {} from algorithm has produced an error: {}", list_iterations[current_grid_point_index as usize].iteration_num, e);
                    current_grid_point = tuning_test.generate_next_grid_point(current_grid_point).unwrap();
                    current_grid_point_index = tuning_test.get_grid_point_index(&current_grid_point).unwrap();
                },
            };

        }

        self.repo.store_results(tuning_test.id.clone(), all_iterations_metrics.clone()).await?;

        let best_index = self.compute_best_config(&all_iterations_metrics, 
                                                     &tuning_test.metrics_weights.unwrap(), 
                                                                      &tuning_test)?;

        let mut best_params = tuning_test.get_current_params(&best_index, initial_params).unwrap();
        format_hashmap(&mut best_params);

        let mut tuning_results_report: HashMap<String, Value> = HashMap::new();
        tuning_results_report.insert(String::from("algo_name"), Value::String(algo.name.clone()));
        tuning_results_report.insert(String::from("dataset_name"), Value::String(self.dataset_service.get_dataset_name_by_id(&tuning_test.dataset_id).await?.unwrap()));
        tuning_results_report.insert(String::from("tested configurations"), Value::Number(all_iterations_metrics.len().into()));
        tuning_results_report.insert(String::from("best_parameters"), Value::Object(best_params.clone().into_iter().collect()));

        self.save_results_to_file("results/grid_search_results.yaml", &tuning_results_report)?;

        Ok(())
    }

    async fn run_random_search(&mut self, tuning_test: &mut TuningConfig) -> Result<(), Box<dyn std::error::Error>> {

        if let Some(start) = tuning_test.dataset_settings.0 {
            self.iteration_service.config.rustle.dataset_start = start;
        }

        if let Some(duration) = tuning_test.dataset_settings.1 {
            self.iteration_service.config.rustle.dataset_duration = duration;
        }

        let mut total_iterations: usize = 0;
        if let Some(TuneType::RandomSearch(rs_config)) = &tuning_test.tuning_type {
            total_iterations = (rs_config.get_param_array_size_by_index(None).unwrap() as usize);
        }

        let mut test_def = TestDefinition{
            id: None,
            name: String::from("dummy_test"),
            workers: 5,
            iterations: total_iterations as u64,
            dataset_name: self.dataset_service.get_dataset_name_by_id(&tuning_test.dataset_id).await?.unwrap(),
            algo_list: vec![
                self.algo_service.get_algo_name_by_id(tuning_test.algo_id.clone()).await?.unwrap(),
            ],
            test_type: TestType::Simple,
            created_at: Utc::now(),
            updated_at: Utc::now(),
        };
        let mut test_exec = TestExecution::new(test_def);
        self.test_exec_service.save_test_execution(&mut test_exec).await?;

        let algo = self.algo_service.get_by_id(tuning_test.algo_id.clone()).await?.unwrap();
        let mut algo_run = self.algo_run_service.create_run(1.0, 
                                                                          total_iterations as u64,
                                                                          &test_exec.id.clone().unwrap(), 
                                                                          &algo.id.clone().unwrap(), 
                                                                          TestType::Simple).await?;

        let list_iterations = self.test_exec_service.execution_repo.get_iterations(&test_exec.id.clone().unwrap()).await?;

        let (msg_tx, mut msg_rx) = mpsc::channel::<ProgressMessage>(100);
        let multi = Arc::new(MultiProgress::new());
        let bars = Arc::new(Mutex::new(HashMap::new()));  

        tokio::spawn({
            let multi = multi.clone();
            let bars = bars.clone();
            async move {
                while let Some(msg) = msg_rx.recv().await {
                    let mut bars = bars.lock().unwrap();
                                    
                    let bar = bars.entry(msg.iteration_num).or_insert_with(|| {
                        let pb = multi.add(ProgressBar::new((msg.total_duration * 1000.0) as u64));
                        pb.set_style(
                            ProgressStyle::with_template(
                                "{elapsed_precise} | {prefix} |> {bar:40.cyan/blue} {percent}% | {pos:.2}/{len:.2} sec"
                            )
                            .unwrap()
                            .progress_chars("███████ "),
                        );
                        pb.set_prefix(format!(
                            "\x1b[93m{} | Iter {}\x1b[0m",
                            msg.algo, msg.iteration_num
                        ));
                        pb
                    });
                                    
                    bar.set_position((msg.duration * 1000.0) as u64);
                }
                                
                // Finish remaining bars
                for bar in bars.lock().unwrap().values() {
                    bar.finish();
                }
            }
        });

        let mut all_iterations_metrics: Vec<(Vec<Metric>, Vec<(String, u64)>)> = Vec::new();
        let mut rng = rand::thread_rng();

        let all_configs: &Vec<HashMap<String, Value>> = &tuning_test.tuning_type.as_ref().unwrap().as_random_search().unwrap().configs;
        let initial_params = &self.params_service.get_by_id(algo.current_params.clone()).await?.unwrap().params;

        let mut writer = Writer::from_path("examples/random_search.csv")?;
        let mut csv_header: Vec<String> = Vec::new();
        let mut csv_header_keys: Vec<String> = Vec::new();

        csv_header.push(String::from("iter"));
        for (key, value) in &tuning_test.parameters_to_tune {
            csv_header.push(key.clone());
            csv_header_keys.push(key.clone());
        }
        csv_header.push(String::from("ape"));
        csv_header.push(String::from("rpe"));
        writer.write_record(&csv_header)?;

        let limit = match tuning_test.time_limit {
            Some(limit) => {
                let time_limit_seconds = (limit * (3600 as f64)) as u64;
                Duration::from_secs(time_limit_seconds)
            },
            None => Duration::from_secs(0),
        };
        let start = Instant::now();

        loop {

            let current_random_point = tuning_test.generate_random_point().unwrap();
            let current_random_index = tuning_test.get_grid_point_index(&current_random_point).unwrap();
            let current_params = tuning_test.get_current_params(&current_random_point, initial_params).unwrap();
            self.params_service.update_params(algo.current_params.clone(), &current_params).await?;

            let iter_job = self.iteration_service.run(list_iterations[current_random_index as usize].clone(), Some(msg_tx.clone())).await;
            match iter_job {
                Ok(_) => {
                    let iter_metrics = self.metrics_service.get_all_iteration_metrics(&list_iterations[current_random_index as usize].id.clone().unwrap()).await.unwrap();
                    all_iterations_metrics.push((iter_metrics.clone(), current_random_point.clone()));

                    let mut current_iter_metrics: (usize, f32, f32) = (0 as usize, 3.0 as f32, 2.0 as f32);
                    if let Some(current_pose_errors) = self.get_pose_error(&iter_metrics) {
                        if let (Some(iter), Some(ape), Some(rpe)) = (tuning_test.get_grid_point_index(&current_random_point), current_pose_errors.ape.rmse, current_pose_errors.rpe.rmse) {
                            current_iter_metrics.0 = iter as usize;
                            current_iter_metrics.1 = ape;
                            current_iter_metrics.2 = rpe;
                            let mut current_iter_data: Vec<String> = Vec::new();
                            current_iter_data.push(current_iter_metrics.0.to_string());
                            for key in &csv_header_keys {
                                current_iter_data.push(current_params.get(key).unwrap().to_string());
                            }
                            current_iter_data.push(current_iter_metrics.1.to_string());
                            current_iter_data.push(current_iter_metrics.2.to_string());
                            tuning_test.save_tuning_progress(&mut writer, &current_iter_metrics, &current_iter_data)?;
                        }
                    }

                    if let Some(_) = tuning_test.time_limit {
                        if start.elapsed() >= limit {
                            println!("Time limit exceed. Saving the results and exiting...");
                            break;   
                        }
                    }
                    else if all_iterations_metrics.len() > tuning_test.early_stopping_params.unwrap().0.clone() as usize {
                        if self.should_tuning_stop(&all_iterations_metrics, &tuning_test.metrics_weights.unwrap(), &tuning_test.early_stopping_params.unwrap()) {
                            println!("Early stopping triggered");
                            break; 
                        } 
                    }
                    else if all_iterations_metrics.len() == total_iterations {
                        break;
                    }
                },
                Err(e) => {
                    //return Err(Box::new(e));
                    warn!("Iteration number {} from algorithm has produced an error: {}", list_iterations[current_random_index as usize].iteration_num, e); 
                },
            };
        }
        
        self.repo.store_results(tuning_test.id.clone(), all_iterations_metrics.clone()).await?;

        let best_index = self.compute_best_config(&all_iterations_metrics, 
                                                     &tuning_test.metrics_weights.unwrap(), 
                                                                      &tuning_test)?;

        let mut best_params = tuning_test.get_current_params(&best_index, initial_params).unwrap();
        format_hashmap(&mut best_params);

        let mut tuning_results_report: HashMap<String, Value> = HashMap::new();
        tuning_results_report.insert(String::from("algo_name"), Value::String(algo.name.clone()));
        tuning_results_report.insert(String::from("dataset_name"), Value::String(self.dataset_service.get_dataset_name_by_id(&tuning_test.dataset_id).await?.unwrap()));
        tuning_results_report.insert(String::from("tested configurations"), Value::Number(all_iterations_metrics.len().into()));
        tuning_results_report.insert(String::from("best_parameters"), Value::Object(best_params.clone().into_iter().collect()));

        self.save_results_to_file("results/random_search_results.yaml", &tuning_results_report)?;

        Ok(())
    }    

    async fn run_simulated_annealing(&mut self, tuning_test: &TuningConfig, sa_config: &mut SimulatedAnnealingConfig) -> Result<(), Box<dyn std::error::Error>> {

        self.iteration_service.config.rustle.dataset_start = 30.0;
        self.iteration_service.config.rustle.dataset_duration = 5.0;

        //let sa_config = tuning_test.tuning_type.clone().unwrap().as_simulated_annealing_as_mut().unwrap();
        //let initial_parameters: HashMap<String, Value> = HashMap::new();
        /*
        let delta_max = 5;

        let mut best_x = 100.0;
        let mut current_x = 20.0;

        let mut best_y = 100.0;
        let mut current_y = 20.0;

        let mut new_x = 0.0;
        let mut new_y = 0.0;

        for i in 0..100 {
            new_x = integer_perturbation(current_x as i64, sa_config.current_temp.clone(), delta_max, -7, 7) as f64;
            new_y = gaussian_perturbation(current_y, 0.5);

            println!("x = {}, y = {}", new_x.clone(), new_y.clone());

            if accept_new_solution(&current_x, &current_y, &new_x, &new_y, &sa_config.current_temp) {
                sa_config.stall_iter_accepted = 0;
                current_x = new_x;
                current_y = new_y; 
                if cost_function(&current_x, &current_y) < cost_function(&best_x, &best_y) {
                    sa_config.stall_iter_best = 0;
                    best_x = current_x;
                    best_y = current_y;
                }
                else {
                    sa_config.stall_iter_best += 1;
                    if sa_config.stall_iter_best > sa_config.stall_iter_best_limit {
                        break;
                    }
                }
            }
            else {
                sa_config.stall_iter_accepted += 1;
                if sa_config.stall_iter_accepted > sa_config.stall_iter_accepted_limit {
                    break;
                }
            }

            sa_config.update_temperature();
            sa_config.update_variables();

        }

        println!("Final solution: f({},{}) -> {}", best_x, best_y, cost_function(&best_x, &best_y));
        */

        let mut test_def = TestDefinition{
            id: None,
            name: String::from("dummy_test"),
            workers: 1,
            iterations: sa_config.max_iterations.clone().unwrap() as u64,
            dataset_name: self.dataset_service.get_dataset_name_by_id(&tuning_test.dataset_id).await?.unwrap(),
            algo_list: vec![
                self.algo_service.get_algo_name_by_id(tuning_test.algo_id.clone()).await?.unwrap(),
            ],
            test_type: TestType::Simple,
            created_at: Utc::now(),
            updated_at: Utc::now(),
        };
        let mut test_exec = TestExecution::new(test_def);
        self.test_exec_service.save_test_execution(&mut test_exec).await?;

        let algo = self.algo_service.get_by_id(tuning_test.algo_id.clone()).await?.unwrap();
        let mut algo_run = self.algo_run_service.create_run(1.0, 
                                                                          sa_config.max_iterations.clone().unwrap() as u64, 
                                                                          &test_exec.id.clone().unwrap(), 
                                                                          &algo.id.clone().unwrap(), 
                                                                          TestType::Simple).await?;

        let list_iterations = self.test_exec_service.execution_repo.get_iterations(&test_exec.id.clone().unwrap()).await?;

        let (msg_tx, mut msg_rx) = mpsc::channel::<ProgressMessage>(100);
        let multi = Arc::new(MultiProgress::new());
        let bars = Arc::new(Mutex::new(HashMap::new()));  
        tokio::spawn({
            let multi = multi.clone();
            let bars = bars.clone();
            async move {
                while let Some(msg) = msg_rx.recv().await {
                    let mut bars = bars.lock().unwrap();
                                    
                    let bar = bars.entry(msg.iteration_num).or_insert_with(|| {
                        let pb = multi.add(ProgressBar::new((msg.total_duration * 1000.0) as u64));
                        pb.set_style(
                            ProgressStyle::with_template(
                                "{elapsed_precise} | {prefix} |> {bar:40.cyan/blue} {percent}% | {pos:.2}/{len:.2} sec"
                            )
                            .unwrap()
                            .progress_chars("███████ "),
                        );
                        pb.set_prefix(format!(
                            "\x1b[93m{} | Iter {}\x1b[0m",
                            msg.algo, msg.iteration_num
                        ));
                        pb
                    });
                                    
                    bar.set_position((msg.duration * 1000.0) as u64);
                }
                                
                // Finish remaining bars
                for bar in bars.lock().unwrap().values() {
                    bar.finish();
                }
            }
        });

        let initial_parameters: HashMap<String, Value> = self.params_service.get_by_id(algo.current_params.clone()).await?.unwrap().params;

        let mut current_parameters = sa_config.get_initial_config(&initial_parameters);
        let mut new_parameters = current_parameters.clone();
        let mut best_parameters: Option<HashMap<String, Value>> = None;
        let mut all_iterations_metrics: Vec<(Vec<Metric>, usize)> = Vec::new();
        //println!("{:?}", initial_parameters.params);

        //let mut current_value = 5.0;
        //initial_parameters.params.insert(String::from("scan_resolution"), json!(current_value));
        //let all_values: Vec<Value> = Vec::new();

        let mut all_things: Vec<f64> = Vec::new();

        //let mut best_value: Option<f64> = None;
        //let mut best_fitness: Option<f64> = None;

        let mut best_things: Option<f64> = None;

        let mut writer = Writer::from_path("examples/simulated_annealing.csv")?;
        let mut csv_header: Vec<String> = Vec::new();
        let mut csv_header_keys: Vec<String> = Vec::new();

        csv_header.push(String::from("iter"));
        if let Some(params_bounds) = &sa_config.parameter_bounds {
            for (key, v) in params_bounds {
                csv_header.push(key.clone());
                csv_header_keys.push(key.clone());
            }
        }
        csv_header.push(String::from("ape"));
        csv_header.push(String::from("rpe"));
        writer.write_record(&csv_header)?;


        for i in 0..sa_config.max_iterations.clone().unwrap() {
            sa_config.update_slam_parameters(&mut current_parameters, &sa_config.parameter_bounds);
            self.params_service.update_params(algo.current_params.clone(), &current_parameters).await?;

            let iter_job = self.iteration_service.run(list_iterations[i].clone(), Some(msg_tx.clone())).await;
            match iter_job {
                Ok(_) => {
                    let iter_metrics = self.metrics_service.get_all_iteration_metrics(&list_iterations[i].id.clone().unwrap()).await.unwrap();
                    all_iterations_metrics.push((iter_metrics.clone(), i));
                    let pose_metrics = self.get_pose_error(&iter_metrics.clone()).unwrap();
                    all_things.push(compute_fitness_function_value(&(pose_metrics.ape.rmse.unwrap(), 
                                                                            pose_metrics.rpe.rmse.unwrap()), 
                                                                            &tuning_test.metrics_weights.clone().unwrap()) as f64);

                    let mut current_iter_metrics: (usize, f32, f32) = (0 as usize, 3.0 as f32, 2.0 as f32);
                    if let Some(current_pose_errors) = self.get_pose_error(&iter_metrics) {
                        if let (current_iter, Some(ape), Some(rpe)) = (i, current_pose_errors.ape.rmse, current_pose_errors.rpe.rmse) {
                            current_iter_metrics.0 = current_iter;
                            current_iter_metrics.1 = ape;
                            current_iter_metrics.2 = rpe;
                            let mut current_iter_data: Vec<String> = Vec::new();
                            current_iter_data.push(current_iter_metrics.0.to_string());
                            for key in &csv_header_keys {
                                current_iter_data.push(current_parameters.get(key).unwrap().to_string());
                            }
                            current_iter_data.push(current_iter_metrics.1.to_string());
                            current_iter_data.push(current_iter_metrics.2.to_string());
                            tuning_test.save_tuning_progress(&mut writer, &current_iter_metrics, &current_iter_data)?;
                        }
                    }

                    if let None = best_things {
                        let pose_metrics = self.get_pose_error(&iter_metrics).unwrap();
                        best_things = Some(compute_fitness_function_value(&(pose_metrics.ape.rmse.unwrap(), pose_metrics.rpe.rmse.unwrap()), &(0.0, 1.0)) as f64);
                    }
                    else {
                        let pose_metrics = self.get_pose_error(&iter_metrics).unwrap();
                        let current_fitness = compute_fitness_function_value(&(pose_metrics.ape.rmse.unwrap(), pose_metrics.rpe.rmse.unwrap()), &(0.0, 1.0));
                        if current_fitness < best_things.unwrap() as f32 {
                            best_things = Some(current_fitness as f64);
                            best_parameters = Some(current_parameters.clone());
                        }
                    }
                }
                Err(e) => {
                    warn!("Iteration number {} from algorithm has produced an error: {}", list_iterations[i as usize].iteration_num, e); 
                }
            }

            sa_config.update_temperature();
            sa_config.update_variables();
        }

        let mut final_best_parameters: HashMap<String, Value> = HashMap::new();
        let mut best_parameters: HashMap<String, Value> = best_parameters.unwrap().clone();
        format_hashmap(&mut best_parameters);
        final_best_parameters.insert(String::from("best_parameters"), json!(best_parameters));

        self.save_results_to_file("examples/sa_test_results.yaml", &final_best_parameters);

        Ok(())
    }

    fn compute_best_config(&self, all_iterations_metrics: &Vec<(Vec<Metric>, Vec<(String, u64)>)>, metrics_weights: &(f32, f32), tuning_config: &TuningConfig) -> Result<Vec<(String, u64)>, Box<dyn std::error::Error>> {
        if all_iterations_metrics.len() == 0 {
            return Err(Box::new(TuningError::NoMetrics()));
        }
        else if all_iterations_metrics.len() == 1 {
            return Ok(all_iterations_metrics[0].1.clone());
        }
        else {
            let mut best_index: Vec<(String, u64)> = all_iterations_metrics[0].1.clone();
            for i in 1..all_iterations_metrics.len() {
                let current_index_relevant_metrics = &self.get_metrics(all_iterations_metrics, &all_iterations_metrics[i].1, tuning_config);
                let best_index_relevant_metrics = &self.get_metrics(all_iterations_metrics, &best_index, tuning_config);

                let current_fitness = compute_fitness_function_value(current_index_relevant_metrics, metrics_weights);
                let best_fitness = compute_fitness_function_value(best_index_relevant_metrics, metrics_weights);

                if current_fitness < best_fitness {
                    best_index = all_iterations_metrics[i].1.clone();
                }
            }
            Ok(best_index)   
        }
    }

    pub fn get_pose_error(&self, iteration_metrics: &Vec<Metric>) -> Option<PoseErrorMetrics> {
        for sub_metric in iteration_metrics.clone() {
            match sub_metric.metric_type {
                PoseError(p) => return Some(p),
                _ => {},
            }
        }
        None
    }

    pub fn get_memory_metrics(&self, iteration_metrics: Vec<Metric>) -> Option<MemoryMetrics> {
        for sub_metric in iteration_metrics.clone() {
            match sub_metric.metric_type {
                Memory(m) => return Some(m),
                _ => {},
            }
        }
        None
    }

    fn get_metrics(&self, all_metrics: &Vec<(Vec<Metric>, Vec<(String, u64)>)>, grid_point: &Vec<(String, u64)>, tuning_config: &TuningConfig) -> (f32, f32) {
        let current_ape = self.get_pose_error(&all_metrics[get_grid_point_index(grid_point, tuning_config).unwrap() as usize].0.clone()).unwrap().ape.rmse.unwrap();
        let current_rpe = self.get_pose_error(&all_metrics[get_grid_point_index(grid_point, tuning_config).unwrap() as usize].0.clone()).unwrap().rpe.rmse.unwrap();

        (current_ape, current_rpe)
    }

    fn compute_metrics(&self, metrics: &Vec<Metric>) -> Option<(f32, f32)> {
        //let pose_errors = self.get_pose_error(metrics);
        if let Some(pose_errors) = self.get_pose_error(metrics) {
            let ape = pose_errors.ape.rmse;
            let rpe = pose_errors.ape.rmse;

            if let (Some(ape), Some(rpe)) = (ape, rpe) {
                Some((ape, rpe))
            }
            else {
                None
            }
        }
        else {
            None
        }
    }

    fn should_tuning_stop(&self, all_metrics: &Vec<(Vec<Metric>, Vec<(String, u64)>)>, metric_weights: &(f32, f32), early_stopping_params: &(u64, f32)) -> bool {

        if all_metrics.len() <= early_stopping_params.0 as usize {
            return false;
        }

        let last_mets_vec = self.get_pose_error(&all_metrics[all_metrics.len() - 1 - (early_stopping_params.0 as usize)].0.clone()).unwrap();
        let last_mets: (f32, f32) = (last_mets_vec.ape.rmse.unwrap(), last_mets_vec.rpe.rmse.unwrap());

        let current_mets_vec = self.get_pose_error(&all_metrics[all_metrics.len() - 1 - (early_stopping_params.0 as usize)].0.clone()).unwrap();
        let current_mets: (f32, f32) = (current_mets_vec.ape.rmse.unwrap(), current_mets_vec.rpe.rmse.unwrap());

        if compute_fitness_function_value(&current_mets, &metric_weights) - compute_fitness_function_value(&last_mets, &metric_weights)
            > early_stopping_params.1 * compute_fitness_function_value(&last_mets, &metric_weights) {
            
            return false;
        }

        return true;
    }

    /// make sure `tuning_report` contains not only the best parameter configuration, but other relevant information(algorithm, dataset, tuning method, evaluated configurations, etc)
    fn save_results_to_file(&self, file_name: &str, tuning_report: &HashMap<String, Value>) -> Result<(), Box<dyn std::error::Error>> {
        let yaml = serde_yaml::to_string(&tuning_report)?;
        let yaml_pretty = yaml.replace("\"[", "[").replace("]\"", "]").replace("\'[", "[").replace("]\'", "]");

        let mut file = File::create(file_name).unwrap();
        file.write_all(yaml_pretty.as_bytes());        

        Ok(())
    }
}

fn extract_numbers(array: &Value) -> Option<Vec<f64>> {
    if let Value::Array(vec) = array {
        let mut result = Vec::new();
        for item in vec {
            if let Value::Number(n) = item {
                result.push(n.as_f64().unwrap());
            } else {
                return None;
            }
        }
        Some(result)
    } else {
        None
    }
}

fn compute_fitness_function_value(metrics: &(f32, f32), metrics_weights: &(f32, f32)) -> f32 {
    metrics.0 * metrics_weights.0 + metrics.1 * metrics_weights.1
}

fn is_index_repeated(all_metrics: &Vec<(Vec<Metric>, usize)>, current_index: usize) -> bool {
    for metric in all_metrics {
        if current_index == metric.1 {
            return true;
        }
    }
    false
}

pub fn get_grid_point_index(grid_point: &Vec<(String, u64)>, tuning_config: &TuningConfig) -> Option<u64> {
    /*
    let mut total_index: u64 = 0;
    for i in 0..random_point.len() {
        total_index += random_point[i].1 * (tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
    }
    //println!("The index is {}", total_index.clone());
    Some(total_index)
    */
    if let Some(TuneType::RandomSearch(rs_config)) = &tuning_config.tuning_type {
        let mut total_index: u64 = 0;
        for i in 0..grid_point.len() {
            total_index += grid_point[i].1 * (rs_config.get_param_array_size_by_index(Some(i)).unwrap()).pow(i as u32);
                //total_index += random_point[i].1 * (rs_config.tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
        }
        Some(total_index)
    }
    else if let Some(TuneType::GridSearch(gs_config)) = &tuning_config.tuning_type {
        let mut total_index: u64 = 0;
        for i in 0..grid_point.len() {
            //total_index += random_point[i].1 * (gs_config.tunable_params_array_sizes.get(&random_point[i].0.clone()).unwrap()).pow(i as u32);
            total_index += grid_point[i].1 * (gs_config.get_param_array_size_by_index(Some(i)).unwrap()).pow(i as u32);
        }
        Some(total_index)
    }
    else { None }
}
