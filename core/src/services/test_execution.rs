use std::path::Path;
use std::{collections::HashMap, fs, sync::Arc};

use charming::Chart;
use charming::{theme::Theme, ImageRenderer};
use chrono::Utc;
use itertools::Itertools;
use log::{info, warn};
use tokio::sync::mpsc::Sender;
use tokio::sync::Mutex;
use yaml_rust2::yaml::Hash;

use crate::db::metric;
use crate::models::metric::{Metric, StatisticalMetrics, MetricType};
use crate::models::metrics::pose_error::{APE, RPE};
use crate::models::metrics::tes::TemporalEfficiencyMetric;
use crate::models::metrics::{PoseErrorMetrics, RobustnessMetric};
use crate::models::test_definitions::{simple, Cut};
use crate::models::test_definitions::test_definition::RobustnessType;
use crate::models::{AlgorithmRun, ProgressMessage};
use crate::services::DbError;
use crate::utils::config::Config;

use crate::utils::plots::{test_adp_chart, test_ape_line_chart, test_memory_usage_line_chart, test_rdp_chart, test_rpe_line_chart};
use crate::{db::{TestExecutionRepo}, models::{metrics::ContainerStats, test_definitions::{test_definition::{TestDefinition, TestType}, CutParams, DropParams}, test_execution::{TestExecution, TestExecutionStatus}, Algorithm, Iteration, SpeedTestParams}, services::error::ProcessingError, utils::plots::test_cpu_load_line_chart
};

use super::{error::PlotError, AlgorithmRunService, IterationService};
use surrealdb::sql::Thing;

#[derive(Clone)]
pub struct TestExecutionService {
    pub execution_repo: TestExecutionRepo,
    algorithm_run_service: AlgorithmRunService,
    iteration_service: IterationService,
}

impl TestExecutionService {
    pub fn new(execution_repo: TestExecutionRepo, algorithm_run_service: AlgorithmRunService, iteration_service: IterationService) -> Self {
        Self { execution_repo, algorithm_run_service, iteration_service}
    }

    pub async fn save_test_execution(&self, exec: &mut TestExecution) -> Result<(), DbError> {

        let _ = self.execution_repo.save(exec).await;

        Ok(())
    }

    pub async fn get_by_name(&self, name: &String) -> Result<Option<TestExecution>, DbError>{
        self.execution_repo.get_by_name(name.to_string()).await
    }

    pub async fn delete_test_by_name(&self, name: &String){
        let _ = self.execution_repo.delete_by_name(name.to_string()).await;
    }

    pub async fn clean_exec(&self, exec: TestExecution)  -> Result<(), DbError>{
        let _ = self.execution_repo.clean_exec(exec).await;
        Ok(())
    }


    pub async fn start_execution(
        &self,
        mut execution: TestExecution,
        msg_tx: Option<Sender<ProgressMessage>>
    ) -> Result<(), ProcessingError> {

        //Update the execution stating that it is starting!
        execution.start_time = Some(Utc::now());
        execution.status = TestExecutionStatus::Running;
        let _ = &self.execution_repo.update_execution(&execution);

        let list_algos = self.execution_repo.get_algos(&execution).await?;

        let execution_id = execution.id.as_ref()  // Get reference to inner Thing
            .ok_or(ProcessingError::General("TestExecution ID".into()))?;

        // Create algorithm runs and their iterations and based on test type
        match &execution.def.test_type {
            TestType::Simple => self.create_simple_runs(&execution, &list_algos).await?,
            TestType::Speed(params) => self.create_speed_runs(&execution, &list_algos, params).await?,
            TestType::Drop(params) => self.create_drop_runs(&execution, &list_algos, params).await?,
            TestType::Cut(params) => self.create_cut_runs(&execution, &list_algos, params).await?
        }

        let list_iterations = self.execution_repo
            .get_iterations(
                &execution_id
            ).await?;

        //Logic To run multiple iterations concurrently.
        let jobs: Arc<Mutex<Vec<Iteration>>> = Arc::new(Mutex::new(list_iterations)); //List of jobs that need to run

        while jobs.lock().await.len() != 0 {
            let results = (0..execution.def.workers).map(|_| async {

                let iteration: Option<Iteration> = jobs.lock().await.pop();
                if let Some(iter) = iteration{

                    //RUN ITERATION JOB
                    // Alterar params
                    let iter_job = self.iteration_service.run(iter.clone(), msg_tx.clone()).await;

                    match iter_job {
                        Ok(_) => (),
                        Err(e) => {
                            warn!("Iteration number {} from algorithm has produced an error: {}", iter.iteration_num, e); 
                        },
                    };

               }
            });
            futures_util::future::join_all(results).await;
        }

        //get all algorithm runs and compute the metrics
        let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await?;
        for algo_run in &algo_run_list{
            self.algorithm_run_service.set_aggregate_metrics(&algo_run).await;
        }

        let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await?;

        let config = Config::load().expect("Unable to Open config");

        // Compute metrics each test
        match &execution.def.test_type {
            TestType::Simple => {
                let hash_algo_run = Self::group_by_algo(algo_run_list);
                let metrics = Self::compute_metrics_simple(hash_algo_run);
                let _ = self.complete_execution(execution.clone(), metrics).await;
            },
            TestType::Speed(_) => {
                let hash_algo_run = Self::group_by_algo(algo_run_list);
                let metrics: HashMap<Algorithm, Vec<Metric>> = Self::compute_metrics_speed(hash_algo_run);

                let _ = self.complete_execution(execution.clone(), metrics).await;
            },
            TestType::Drop(drop_params) => {
                let mut simple_algo_run_list: Vec<AlgorithmRun> = vec![];

                let dataset = self.execution_repo.get_dataset_by_name(&execution.def.dataset_name).await.unwrap();

                for algo in list_algos{
                    let simple_tests = self.execution_repo.get_by_algo_dataset_type(&algo.name, &execution.def.dataset_name, "simple").await;
                    let simple_exec = match simple_tests {
                        Ok(s) =>{
                            s.into_iter().max_by_key(|td| td.def.iterations).unwrap()
                        },
                        Err(_) => {
                            execution.clone()
                        },
                    };

                    let simple_algo_run_vec = self.execution_repo.get_algorithm_runs(&simple_exec.id.clone().unwrap()).await.unwrap();

                    let simple_algo_run: AlgorithmRun = simple_algo_run_vec
                        .into_iter()
                        .filter(|run| run.algo.name == algo.name)
                        .filter(|run| run.test_type.as_str() == "simple")
                        .last().unwrap();

                    simple_algo_run_list.push(simple_algo_run);
                }

                let metrics = Self::compute_metrics_drop(
                    algo_run_list, 
                    simple_algo_run_list, 
                    drop_params, 
                    dataset.duration,
                    config.rustle.time_precision
                );

                let _ = self.complete_execution(execution.clone(), metrics).await;

            },
            TestType::Cut(cut_params) => {

                let mut simple_algo_run_list: Vec<AlgorithmRun> = vec![];

                let dataset = self.execution_repo.get_dataset_by_name(&execution.def.dataset_name).await.unwrap();

                for algo in list_algos{
                    let simple_tests = self.execution_repo.get_by_algo_dataset_type(&algo.name, &execution.def.dataset_name, "simple").await;
                    let simple_exec = match simple_tests {
                        Ok(s) =>{
                            s.into_iter().max_by_key(|td| td.def.iterations).unwrap()
                        },
                        Err(_) => {
                            execution.clone()
                        },
                    };

                    //get the test definition with the highest amount of iterations
                    let algo_run_list = self.execution_repo.get_algorithm_runs(&simple_exec.id.clone().unwrap()).await.unwrap();

                    let simple_algo_run: AlgorithmRun = algo_run_list
                        .into_iter()
                        .filter(|run| run.algo.name == algo.name)
                        .filter(|run| run.test_type.as_str() == "simple")
                        .last().unwrap();

                    simple_algo_run_list.push(simple_algo_run);
                }

                let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await.unwrap();

                let metrics = Self::compute_metrics_cut(
                    algo_run_list, 
                    simple_algo_run_list, 
                    cut_params, 
                    dataset.duration,
                    config.rustle.time_precision
                );

                let _ = self.complete_execution(execution.clone(), metrics).await.unwrap();

            },
        }

        Ok(())
    }

    pub async fn plot_execution(
        &self,
        execution: &TestExecution,
        path: &str,
        overwrite: bool,
        format:  &str
    ) -> Result<(), PlotError> {

        let execution_id = execution.clone().id.ok_or(PlotError::MissingData("Test run ID is missing, probably was never run".to_owned()))?;
        let config = Config::load().expect("Unable to load configuration.");
        let iterations = self.execution_repo.get_iterations(&execution_id).await.map_err(|_e| PlotError::MissingData(format!("No iterations found for test {}. Did you run the test?", execution.def.name)))?;
        
        // Plot for each iteration!
        for iter in iterations{

            match self.iteration_service.plot(iter, &execution.def.test_type, path, overwrite, format).await{
                Ok(hash_plots) => {
                    for (p, ch) in hash_plots{

                        let mut renderer = ImageRenderer::new(config.plotting.width, config.plotting.height).theme(Theme::Infographic);
                        let _ = renderer.save(&ch, p);
                    }
                },
                Err(e) => match e {
                    PlotError::MissingData(e) => warn!("Error when plotting for iteration: {e}"),
                    PlotError::FileExists(_) => warn!("File already exist, use the --overwrite flag"),
                },
            }

        }

        //Plot for each Algorithm
        let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await.map_err(|_err| PlotError::MissingData("Test run was not found".to_owned()))?;
        for algo_run in &algo_run_list{

            match self.algorithm_run_service.plot(algo_run, path, overwrite, format, &config).await{
                Ok(hash_plots) => {
                    for (p, ch) in hash_plots{

                        let mut renderer = ImageRenderer::new(config.plotting.width, config.plotting.height).theme(Theme::Infographic);
                        let _ = renderer.save(&ch, p);
                    }
                },
                Err(_) => (),
            }
        }

        //Plot for the Different Tests
        let charts = self.plot(execution.clone(), &algo_run_list, path, overwrite, format, &config).await?;
        for (p, ch) in charts{
            let mut renderer = ImageRenderer::new(config.plotting.width, config.plotting.height).theme(Theme::Infographic);
            let _ = renderer.save(&ch, p);
        }

        Ok(())
    }

    pub async fn get_iterations_by_algo_run(&self, run: AlgorithmRun) -> Result<Vec<Iteration>, ProcessingError>{
        let iterations = self.algorithm_run_service.get_iterations(&run).await;
        iterations.map_err(|_e| ProcessingError::InvalidIteration("iterations were not found".to_owned()))
    }

    pub async fn get_metrics_by_iteration(&self, iter: Iteration) -> Result<Vec<Metric>, ProcessingError>{

        let metrics = self.iteration_service.get_metrics(&iter).await;
        let m = metrics.map_err(|_e| ProcessingError::General(format!("Metric for iteration {:?} is missing", iter)));

        m
    }

    async fn create_simple_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
    ) -> Result<(), ProcessingError> {


        for algorithm in algo_list {

            self.algorithm_run_service.create_run(
                1.0,
                execution.def.iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                execution.def.test_type.clone()
            ).await?;

        }
        Ok(())
    }

    async fn create_speed_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
        params: &SpeedTestParams,
    ) -> Result<(), ProcessingError> {
        // Implementation for speed bag tests
        // Create multiple runs with different parameters
        // Example: 1 run per speed setting
        for speed_setting in &params.speed_factors {
            for algorithm in algo_list {

                self.algorithm_run_service.create_run(
                    *speed_setting,
                    execution.def.iterations,
                    &execution.id.as_ref().unwrap(),
                    &algorithm.id.clone().unwrap(),
                    execution.def.test_type.clone()
                ).await?;
            }
        }
        Ok(())
    }

    async fn create_drop_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
        _params: &DropParams,
    ) -> Result<(), ProcessingError> {
        
        for algorithm in algo_list {

            //Check if there is any Simple run with the algorithm in the same dataset. If not Create a new Simple Run, with the 
            // same parameters...

            self.algorithm_run_service.create_run(
                1.0, 
                execution.def.iterations, 
                &execution.id.as_ref().unwrap(), 
                &algorithm.id.clone().unwrap(), 
                TestType::Simple
            ).await?;
            
            //TODO, Now I ran a simple test at the same time as the cut/drop. 
            //But if a simple test is already available I should just use it. Need to fix the matching timestamps problem
            //let simple_tests = self.definition_repo.get_by_algo_dataset_type(&algorithm.name, &execution.dataset_name, "simple").await;
            //match simple_tests{
            //    Ok(_) => (),
            //    Err(_) => {
            //        //If does not exist create a "simple" algorithm run for performance comparison!
            //        self.algorithm_run_service.create_run(
            //            1.0, 
            //            execution.def.iterations, 
            //            &execution.id.as_ref().unwrap(), 
            //            &algorithm.id.clone().unwrap(), 
            //            "simple"
            //        ).await?;
            //    },
            //};

            self.algorithm_run_service.create_run(
                1.0,
                execution.def.iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                execution.def.test_type.clone()
            ).await?;

        }

        Ok(())
    }

    async fn create_cut_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
        _params: &CutParams,
    ) -> Result<(), ProcessingError> {
        
        for algorithm in algo_list {

            self.algorithm_run_service.create_run(
                1.0, 
                execution.def.iterations, 
                &execution.id.as_ref().unwrap(), 
                &algorithm.id.clone().unwrap(), 
                TestType::Simple
            ).await?;

            //TODO, Now I ran a simple test at the same time as the cut/drop. 
            //But if a simple test is already available I should just use it. Need to fix the matching timestamps problem
            //let simple_tests = self.definition_repo.get_by_algo_dataset_type(&algorithm.name, &execution.dataset_name, "simple").await;
            //match simple_tests{
            //    Ok(_) => (),
            //    Err(_) => {
            //        //If does not exist create a "simple" algorithm run for performance comparison!
            //        self.algorithm_run_service.create_run(
            //            1.0, 
            //            execution.def.iterations, 
            //            &execution.id.as_ref().unwrap(), 
            //            &algorithm.id.clone().unwrap(), 
            //            "simple"
            //        ).await?;
            //    },
            //};
            self.algorithm_run_service.create_run(
                1.0,
                execution.def.iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                execution.def.test_type.clone()
            ).await?;

        }

        Ok(())
    }

    pub async fn complete_execution(
        &self,
        mut execution: TestExecution,
        metrics: HashMap<Algorithm, Vec<Metric>>
    ) -> Result<(), ProcessingError> {
        execution.status = TestExecutionStatus::Completed;
        execution.end_time = Some(Utc::now());

        let mut hash_metric:HashMap<String, Vec<Metric>> = HashMap::new();

        metrics.into_iter().for_each(|(al, v)|{
            hash_metric.insert(
                al.name,
                 v.clone()
                );
        });

        execution.metrics = hash_metric;
        
        let _ = self.execution_repo.update_execution(&execution).await.unwrap();
        Ok(())
    }


    pub async fn plot(&self, exec: TestExecution, algo_run_list: &Vec<AlgorithmRun>, path: &str, overwrite: bool, format:  &str, config: &Config) -> Result<HashMap<String, Chart>, PlotError>{

        let mut hash: HashMap<String, Chart> = HashMap::new();

        let exec_thing: &Thing = exec.id.as_ref().unwrap();
        let te_str = exec_thing.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
        let full_path = format!("{path}/{te_str}");

        //Create the directories if they dont exist
        fs::create_dir_all(&full_path).unwrap();

        // Call the other plots
        let mut files = vec!["test_cpu_load", "test_memory_usage", "test_ape", "test_rpe"];

        match &exec.def.test_type{
            TestType::Simple => (),
            TestType::Speed(_) => (),
            TestType::Drop(_) => {
                files.push("test_drop")
            },
            TestType::Cut(_) => {
                files.push("test_cut")
            },
        }

        for f in files{

            let filepath = format!("{}/{}.{}", full_path, f, format);

            if Path::new(&filepath).exists() && !overwrite {
                //Not sure if I should return here, or just emit a warning
                return Err(PlotError::FileExists(filepath));
            } else {
                let chart = match f {
                    "test_cpu_load" => {
                        let chart = self.plot_cpu_load(&algo_run_list, config).await;
                        hash.insert(filepath, chart?);
                    },
                    "test_memory_usage" => {
                        let chart = self.plot_memory_usage(&algo_run_list, config).await;
                        hash.insert(filepath, chart?);
                    },
                    "test_ape" => {
                        let chart = self.plot_ape(&algo_run_list, config).await;
                        hash.insert(filepath, chart?);
                    },
                    "test_rpe" => {
                        let chart = self.plot_rpe(&algo_run_list, config).await;
                        hash.insert(filepath, chart?);
                    },
                    "test_drop" => {
                        let filepath_adp = format!("{}/{}_adp.{}", full_path, f, format);
                        let filepath_rdp = format!("{}/{}_rdp.{}", full_path, f, format);

                        let mut hash_algo_metric: HashMap<Algorithm, &Vec<Metric>> = HashMap::new();


                        for (id, metrics) in &exec.metrics{
                            let algo = self.execution_repo
                                .get_algorithm_by_name(id).await
                                .map_err(|e| PlotError::MissingData(format!("Missing algorithm {:#?}", e)))?;

                            hash_algo_metric.insert(algo, metrics);
                        }

                        let adp_chart = test_adp_chart(&hash_algo_metric, &exec.def.test_type, config);
                        let rdp_chart = test_rdp_chart(&hash_algo_metric, &exec.def.test_type, config);

                        hash.insert(filepath_adp, adp_chart?);
                        hash.insert(filepath_rdp, rdp_chart?);


                    },
                    "test_cut" => {
                        let filepath_adp = format!("{}/{}_adp.{}", full_path, f, format);
                        let filepath_rdp = format!("{}/{}_rdp.{}", full_path, f, format);

                        let mut hash_algo_metric: HashMap<Algorithm, &Vec<Metric>> = HashMap::new();


                        for (id, metrics) in &exec.metrics{
                            let algo = self.execution_repo
                                .get_algorithm_by_name(id).await
                                .map_err(|e| PlotError::MissingData(format!("Missing algorithm {:#?}", e)))?;

                            hash_algo_metric.insert(algo, metrics);
                        }

                        let adp_chart = test_adp_chart(&hash_algo_metric, &exec.def.test_type, config);
                        let rdp_chart = test_rdp_chart(&hash_algo_metric, &exec.def.test_type, config);

                        hash.insert(filepath_adp, adp_chart?);
                        hash.insert(filepath_rdp, rdp_chart?);


                    },
                    &_ => todo!()
                };
            }
        }
        
        Ok(hash)
    }

    pub async fn plot_cpu_load(&self, algo_run_list: &Vec<AlgorithmRun>, config: &Config) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_cs_hashmap: HashMap<AlgorithmRun, Vec<Vec<ContainerStats>>> = HashMap::new();

        for algo_run in algo_run_list{
            
            //For each AlgorithmRun I need the container stats
            let container_stats = self.algorithm_run_service.get_all_container_stats(&algo_run).await;

            if !container_stats.is_empty(){
                algo_cs_hashmap.insert(algo_run.clone(), container_stats);
            }


        };

        let cpu_chart = test_cpu_load_line_chart(&algo_cs_hashmap, config);

        cpu_chart
    }

    pub async fn plot_memory_usage(&self, algo_run_list: &Vec<AlgorithmRun>, config: &Config) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_cs_hashmap: HashMap<AlgorithmRun, Vec<Vec<ContainerStats>>> = HashMap::new();

        for algo_run in algo_run_list{
            
            //For each AlgorithmRun I need the container stats
            let container_stats = self.algorithm_run_service.get_all_container_stats(&algo_run).await;

            if !container_stats.is_empty(){
                algo_cs_hashmap.insert(algo_run.clone(), container_stats);
            }


        };

        let mem_chart = test_memory_usage_line_chart(&algo_cs_hashmap, config);

        mem_chart
    }

    pub async fn plot_ape(&self, algo_run_list: &Vec<AlgorithmRun>, config: &Config) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_ape_hashmap: HashMap<AlgorithmRun, Vec<Vec<APE>>> = HashMap::new();

        for algo_run in algo_run_list{
            
            //For each AlgorithmRun I need the container stats
            let ape_list: Vec<Vec<APE>> = self.algorithm_run_service.get_all_ape(&algo_run).await;

            if !ape_list.is_empty(){
                algo_ape_hashmap.insert(algo_run.clone(), ape_list);
            }


        };

        let ape_chart = test_ape_line_chart(&algo_ape_hashmap, config);

        ape_chart
    }

    pub async fn plot_rpe(&self, algo_run_list: &Vec<AlgorithmRun>, config: &Config) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_rpe_hashmap: HashMap<AlgorithmRun, Vec<Vec<RPE>>> = HashMap::new();

        for algo_run in algo_run_list{
            
            //For each AlgorithmRun I need the container stats
            let rpe_list: Vec<Vec<RPE>> = self.algorithm_run_service.get_all_rpe(&algo_run).await;

            if !rpe_list.is_empty(){
                algo_rpe_hashmap.insert(algo_run.clone(), rpe_list);
            }

        };

        let rpe_chart = test_rpe_line_chart(&algo_rpe_hashmap, config);

        rpe_chart
    }

    pub async fn get_all(&self) -> Result<Vec<TestExecution>, ProcessingError> {
        let results = self.execution_repo.list_all().await?;
        Ok(results)
    }

    pub async fn get_algo_runs(&self, test_execution_id: &Thing) -> Result<Vec<AlgorithmRun>, ProcessingError>{
        let results = self.execution_repo.get_algorithm_runs(test_execution_id).await?;
        Ok(results)
    }


    fn compute_metrics_simple(list: HashMap<Algorithm, Vec<AlgorithmRun>>) -> HashMap<Algorithm, Vec<Metric>>{

        let mut algo_metric: HashMap<Algorithm, Vec<Metric>> = HashMap::new();

        let _ = list.into_iter()
            .for_each(|(k, v)|{

                algo_metric.insert(k, v[0].metrics.clone());

            });

        algo_metric
    }


    fn compute_metrics_speed(list: HashMap<Algorithm, Vec<AlgorithmRun>>) -> HashMap<Algorithm, Vec<Metric>>{

        let mut algo_metric: HashMap<Algorithm, Vec<Metric>> = HashMap::new();

        let _ = list.into_iter()
            .for_each(|(k,v)|{

                let mut speed_freq: HashMap<String, StatisticalMetrics> = HashMap::new();
                let mut speed_pose: HashMap<String, PoseErrorMetrics>   = HashMap::new();

                for algo_run in v {

                    for metric in algo_run.metrics{
                        match metric.metric_type {
                            crate::models::metric::MetricType::PoseError(pose_error_metrics) => {
                                                        speed_pose.insert(algo_run.bag_speed.to_string(), pose_error_metrics);
                                                    },
                            crate::models::metric::MetricType::Frequency(statistical_metrics) => {
                                                        speed_freq.insert(algo_run.bag_speed.to_string(), statistical_metrics);
                                                    },
                            _ => (),
                        }
                    }
                }

                //Create a new metric
                let metric = Metric { 
                    id: None, 
                    metric_type: crate::models::metric::MetricType::TemporalEfficiency(
                        TemporalEfficiencyMetric::new(speed_freq, speed_pose).unwrap()
                    )
                };

                algo_metric.insert(k, vec![metric]);

            }
        );

        algo_metric

    }

    fn compute_metrics_drop(
        drop_vec: Vec<AlgorithmRun>,
        simple_vec: Vec<AlgorithmRun>,
        drop_params: &DropParams,
        duration: Option<f32>,
        precision: f32
    ) -> HashMap<Algorithm, Vec<Metric>> {

        let mut algo_metric: HashMap<Algorithm, Vec<Metric>> = HashMap::new();

        let duration = duration.unwrap();

        let simple_map: HashMap<Algorithm, &AlgorithmRun> = simple_vec
            .iter()
            .map(|run| (run.algo.clone(), run))
            .collect();

        let drop_vec: Vec<AlgorithmRun> = drop_vec.into_iter()
            .filter(|ar| match ar.test_type {
                TestType::Drop(_) => true,
                _ => false
            })
            .collect();

        for drop_run in drop_vec {
            if let Some(simple_run) = simple_map.get(&drop_run.algo) {

                let metric = Metric { 
                    id: None, 
                    metric_type: MetricType::Robustness(
                        RobustnessMetric::new(
                            RobustnessType::Drop(drop_params.drop_list.clone()),
                            duration,
                            &drop_run,
                            &simple_run,
                            precision
                        ).unwrap()
                    )
                };

                algo_metric.insert(drop_run.algo.clone(), vec![metric]);
            }
        }

        algo_metric
    }

    fn compute_metrics_cut(
        cut_vec: Vec<AlgorithmRun>,
        simple_vec: Vec<AlgorithmRun>,
        cut_params: &CutParams,
        duration: Option<f32>,
        precision: f32
    ) -> HashMap<Algorithm, Vec<Metric>> {

        let mut algo_metric: HashMap<Algorithm, Vec<Metric>> = HashMap::new();

        let duration = duration.unwrap();

        let simple_map: HashMap<Algorithm, &AlgorithmRun> = simple_vec
            .iter()
            .map(|run| (run.algo.clone(), run))
            .collect();

        let cut_vec: Vec<AlgorithmRun> = cut_vec.into_iter()
            .filter(|ar| match ar.test_type {
                TestType::Cut(_) => true,
                _ => false
            })
            .collect();

        for cut_run in cut_vec {
            if let Some(simple_run) = simple_map.get(&cut_run.algo) {

                let metric = Metric { 
                    id: None, 
                    metric_type: MetricType::Robustness(
                        RobustnessMetric::new(
                            RobustnessType::Cut(cut_params.cut_list.clone()),
                            duration,
                            &cut_run,
                            &simple_run,
                            precision
                        ).unwrap()
                    )
                };

                algo_metric.insert(cut_run.algo.clone(), vec![metric]);
            }
        }

        algo_metric
    }


    fn group_by_algo(runs: Vec<AlgorithmRun>) -> HashMap<Algorithm, Vec<AlgorithmRun>> {
        let mut grouped: HashMap<Algorithm, Vec<AlgorithmRun>> = HashMap::new();
    
        for run in runs {
            grouped.entry(run.algo.clone()) // clone the key if necessary
                .or_insert_with(Vec::new)
                .push(run);
        }
    
        grouped
    }


}