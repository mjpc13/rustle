use std::path::Path;
use std::{collections::HashMap, fs, sync::Arc};

use bollard::exec;
use charming::Chart;
use charming::{theme::Theme, ImageRenderer};
use chrono::Utc;
use directories::ProjectDirs;
use log::{debug, info, warn};
use tokio::sync::Mutex;

use crate::models::metric::Metric;
use crate::models::metrics::pose_error::{APE, RPE};
use crate::models::AlgorithmRun;
use crate::utils::config::Config;

use crate::utils::plots::{test_ape_line_chart, test_memory_usage_line_chart, test_rpe_line_chart};
use crate::{db::{TestDefinitionRepo, TestExecutionRepo}, models::{metrics::ContainerStats, test_definitions::{test_definition::{TestDefinition, TestType}, CutParams, DropParams}, test_execution::{TestExecution, TestExecutionStatus}, Algorithm, Iteration, SpeedTestParams, TestResults}, services::error::ProcessingError, utils::plots::test_cpu_load_line_chart
};

use super::{error::{ExecutionError, PlotError, RunError}, AlgorithmRunService, IterationService};
use surrealdb::sql::Thing;

pub struct TestExecutionService {
    execution_repo: TestExecutionRepo,
    definition_repo: TestDefinitionRepo,
    algorithm_run_service: AlgorithmRunService,
    iteration_service: IterationService,
}

impl TestExecutionService {
    pub fn new(execution_repo: TestExecutionRepo, definition_repo: TestDefinitionRepo, algorithm_run_service: AlgorithmRunService, iteration_service: IterationService) -> Self {
        Self { execution_repo, definition_repo, algorithm_run_service, iteration_service}
    }

    //THIS IS MY TASK BATCH!!!
    pub async fn start_execution(
        &self,
        mut execution: TestExecution,
        def: &TestDefinition
    ) -> Result<TestExecution, ProcessingError> {


        let _ = &self.execution_repo.save(&mut execution, def).await;

        let list_algos = self.execution_repo.get_algos(&execution).await?;

        let execution_id = execution.id.as_ref()  // Get reference to inner Thing
            .ok_or(ProcessingError::General("TestExecution ID".into()))?;

        // Create algorithm runs and their iterations and based on test type
        match &def.test_type {
            TestType::Simple => self.create_simple_runs(&execution, &list_algos).await?,
            TestType::Speed(params) => self.create_speedbag_runs(&execution, &list_algos, params).await?,
            TestType::Drop(params) => self.create_drop_runs(&execution, &list_algos, params).await?,
            TestType::Cut(params) => self.create_cut_runs(&execution, &list_algos, params).await?
        }

        let mut list_iterations = self.execution_repo
            .get_iterations(
                &execution_id
            ).await?;

        //Logic To run multiple iterations concurrently.
        let jobs: Arc<Mutex<Vec<Iteration>>> = Arc::new(Mutex::new(list_iterations)); //List of jobs that need to run

        while jobs.lock().await.len() != 0 {
            let results = (0..def.workers).map(|_| async {

                let mut iteration: Option<Iteration> = jobs.lock().await.pop();
                if let Some(iter) = iteration{

                    //RUN ITERATION JOB
                    let _ = self.iteration_service.run(iter).await;

               }
            });
            futures_util::future::join_all(results).await;
        }


        //get all algorithm runs
        let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await?;
        for algo_run in algo_run_list{
            self.algorithm_run_service.set_aggregate_metrics(&algo_run).await;
        }

        Ok(execution)
    }


    pub async fn plot_execution(
        &self,
        def: &TestDefinition,
        path: &str,
        overwrite: bool,
        format:  &str
    ) -> Result<(), PlotError> {

        let execution: TestExecution = self.definition_repo.get_test_executions(def).await.map_err(|_err| PlotError::MissingData("Test run was not found".to_owned()))?;
        let execution_id = execution.clone().id.ok_or(PlotError::MissingData("Test run ID is missing, probably was never run".to_owned()))?;

        let config = Config::load().expect("Unable to load configuration.");

        let iterations = self.execution_repo.get_iterations(&execution_id).await.map_err(|_e| PlotError::MissingData(format!("No iterations found for test {}. Did you run the test?", def.name)))?;
        // Plot for each iteration!
        for iter in iterations{
            let hash_plots = self.iteration_service.plot(iter, def, path, overwrite, format).await?;

            for (p, ch) in hash_plots{
                // The Theme and shape can be in the Config file!
                let mut renderer = ImageRenderer::new(1000, 1000).theme(Theme::Infographic);
                let _ = renderer.save(&ch, p);
            }
        }

        
        //get all algorithm runs
        let algo_run_list = self.execution_repo.get_algorithm_runs(&execution_id).await.map_err(|_err| PlotError::MissingData("Test run was not found".to_owned()))?;
        for algo_run in &algo_run_list{
            self.algorithm_run_service.set_aggregate_metrics(algo_run).await;

            let hash_plots = self.algorithm_run_service.plot(algo_run, path, overwrite, format).await?;

            for (p, ch) in hash_plots{

                // The Theme and shape can be in the Config file!
                let mut renderer = ImageRenderer::new(config.plotting.width, config.plotting.height).theme(Theme::Infographic);
                let _ = renderer.save(&ch, p);
            }
        }

        let charts = self.plot(execution, &algo_run_list, path, overwrite, format).await?;
        for (p, ch) in charts{

            // The Theme and shape can be in the Config file!
            let mut renderer = ImageRenderer::new(config.plotting.width, config.plotting.height).theme(Theme::Infographic);
            let _ = renderer.save(&ch, p);
        }

        Ok(())
    }




    pub async fn get_execution_results(
        &self,
        def: &TestDefinition,
    ) -> Result<(), PlotError>{

        todo!("Get executions results for a test definition is not implemented yet!");

        return Ok(())
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
                execution.num_iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                "simple"
            ).await?;

        }
        Ok(())
    }

    async fn create_speedbag_runs(
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
                    execution.num_iterations,
                    &execution.id.as_ref().unwrap(),
                    &algorithm.id.clone().unwrap(),
                    "speed"
                ).await?;
            }
        }
        Ok(())
    }

    async fn create_drop_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
        params: &DropParams,
    ) -> Result<(), ProcessingError> {
        
        //Migh be better if I pass the Degradations params in here
        for algorithm in algo_list {

            self.algorithm_run_service.create_run(
                1.0,
                execution.num_iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                "drop"
            ).await?;

        }

        Ok(())
    }

    async fn create_cut_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
        params: &CutParams,
    ) -> Result<(), ProcessingError> {
        
        //Migh be better if I pass the Degradations params in here
        for algorithm in algo_list {

            self.algorithm_run_service.create_run(
                1.0,
                execution.num_iterations,
                &execution.id.as_ref().unwrap(),
                &algorithm.id.clone().unwrap(),
                "cut"
            ).await?;

        }

        Ok(())
    }

    pub async fn complete_execution(
        &self,
        mut execution: TestExecution,
        results: TestResults
    ) -> Result<(), ProcessingError> {
        execution.status = TestExecutionStatus::Completed;
        execution.end_time = Some(Utc::now());
        execution.results = Some(results);
        
        //self.execution_repo.save(&execution).await?;
        Ok(())
    }



    pub async fn plot(&self, exec: TestExecution, algo_run_list: &Vec<AlgorithmRun>, path: &str, overwrite: bool, format:  &str) -> Result<HashMap<String, Chart>, PlotError>{

        let mut hash: HashMap<String, Chart> = HashMap::new();

        let exec_thing: Thing = exec.id.unwrap();
        let te_str = exec_thing.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
        let full_path = format!("{path}/{te_str}"); //add the Thing of TestExecution here!!!!

        //Create the directories if they dont exist
        fs::create_dir_all(&full_path).unwrap();

        // Call the other plots
        let files = ["test_cpu_load", "test_memory_usage", "test_ape", "test_rpe"];

        for f in files{

            let filepath = format!("{}/{}.{}", full_path, f, format);

            if Path::new(&filepath).exists() && !overwrite {
                //Not sure if I should return here, or just emit a warning
                return Err(PlotError::FileExists(filepath));
            } else {
                let chart = match f {
                    "test_cpu_load" => self.plot_cpu_load(&algo_run_list).await,
                    "test_memory_usage" => self.plot_memory_usage(&algo_run_list).await,
                    "test_ape" => self.plot_ape(&algo_run_list).await,
                    "test_rpe" => self.plot_rpe(&algo_run_list).await,
                    &_ => todo!()
                };
                hash.insert(filepath, chart?);
            }
        }
        
        Ok(hash)
    }



    pub async fn plot_cpu_load(&self, algo_run_list: &Vec<AlgorithmRun>) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_cs_hashmap: HashMap<Algorithm, Vec<Vec<ContainerStats>>> = HashMap::new();

        for algo_run in algo_run_list{
            let algo = self.algorithm_run_service.get_algorithm(&algo_run).await.unwrap();
            
            //For each AlgorithmRun I need the container stats
            let container_stats = self.algorithm_run_service.get_all_container_stats(&algo_run).await;

            algo_cs_hashmap.insert(algo, container_stats);

        };

        let cpu_chart = test_cpu_load_line_chart(&algo_cs_hashmap);

        cpu_chart
    }

    pub async fn plot_memory_usage(&self, algo_run_list: &Vec<AlgorithmRun>) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_cs_hashmap: HashMap<Algorithm, Vec<Vec<ContainerStats>>> = HashMap::new();

        for algo_run in algo_run_list{
            let algo = self.algorithm_run_service.get_algorithm(&algo_run).await.unwrap();
            
            //For each AlgorithmRun I need the container stats
            let container_stats = self.algorithm_run_service.get_all_container_stats(&algo_run).await;

            algo_cs_hashmap.insert(algo, container_stats);

        };

        let mem_chart = test_memory_usage_line_chart(&algo_cs_hashmap);

        mem_chart
    }

    pub async fn plot_ape(&self, algo_run_list: &Vec<AlgorithmRun>) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_ape_hashmap: HashMap<Algorithm, Vec<Vec<APE>>> = HashMap::new();

        for algo_run in algo_run_list{
            let algo = self.algorithm_run_service.get_algorithm(&algo_run).await.unwrap();
            
            //For each AlgorithmRun I need the container stats
            let ape_list: Vec<Vec<APE>> = self.algorithm_run_service.get_all_ape(&algo_run).await;


            algo_ape_hashmap.insert(algo, ape_list);

        };

        let ape_chart = test_ape_line_chart(&algo_ape_hashmap);

        ape_chart
    }

    pub async fn plot_rpe(&self, algo_run_list: &Vec<AlgorithmRun>) -> Result<Chart, PlotError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>
        let mut algo_rpe_hashmap: HashMap<Algorithm, Vec<Vec<RPE>>> = HashMap::new();

        for algo_run in algo_run_list{
            let algo = self.algorithm_run_service.get_algorithm(&algo_run).await.unwrap();
            
            //For each AlgorithmRun I need the container stats
            let rpe_list: Vec<Vec<RPE>> = self.algorithm_run_service.get_all_rpe(&algo_run).await;


            algo_rpe_hashmap.insert(algo, rpe_list);

        };

        let rpe_chart = test_rpe_line_chart(&algo_rpe_hashmap);

        rpe_chart
    }



    pub async fn get_all(&self) -> Result<Vec<TestExecution>, ProcessingError> {
        let results = self.execution_repo.list_all().await?;
        Ok(results)
    }

    pub async fn delete_test_by_name(&self, name: &String){
        self.execution_repo.delete_by_name(name.to_string()).await;
    }

    pub async fn get_algo_runs(&self, test_execution_id: &Thing) -> Result<Vec<AlgorithmRun>, ProcessingError>{
        let results = self.execution_repo.get_algorithm_runs(test_execution_id).await?;
        Ok(results)
    }


}