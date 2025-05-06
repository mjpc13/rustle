use std::{collections::HashMap, fs, sync::Arc};

use chrono::Utc;
use directories::ProjectDirs;
use log::{debug, info, warn};
use tokio::sync::Mutex;

use crate::{
    db::{test_execution, TestDefinitionRepo, TestExecutionRepo}, 
    models::{metrics::ContainerStats, test_definitions::{test_definition::{TestDefinition, TestType}, CutParams, DropParams}, test_execution::{TestExecution, TestExecutionStatus}, Algorithm, AlgorithmRun, Iteration, SimpleTestParams, SpeedTestParams, TestResults}, services::error::ProcessingError, utils::plots::test_cpu_load_line_chart
};

use super::{error::{ExecutionError, RunError}, AlgorithmRunService, DatasetService, IterationService};
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
        let algo_run_list = self.execution_repo.get_algorithm_runs(execution_id).await?;
        for algo_run in algo_run_list{
            self.algorithm_run_service.set_aggregate_metrics(&algo_run).await;
            // Plot things for the iterations!!!
            self.algorithm_run_service.plot_cpu_load(&algo_run).await;
            self.algorithm_run_service.plot_memory_usage(&algo_run).await;
            self.algorithm_run_service.plot_ape(&algo_run).await;
        }

        //PLOT ALGORITHMS AGAINST EACH OTHER!!!

        //Need to get the algorithm and the algorithm run.
        let _ = self.plot_cpu_load(execution_id).await;


        Ok(execution)
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


    pub async fn plot_cpu_load(&self, test_execution_id: &Thing) -> Result<(), ExecutionError>{

        //get algorithms and algo runs and build a Hashmap<Algorithm, Vec<Vec<ContainerStats>>>

        let mut algo_cs_hashmap: HashMap<Algorithm, Vec<Vec<ContainerStats>>> = HashMap::new();

        //Get list of algorithm_run.
        let algo_run_list = self.execution_repo.get_algorithm_runs(test_execution_id).await.unwrap();

        for algo_run in algo_run_list{
            let algo = self.algorithm_run_service.get_algorithm(&algo_run).await.unwrap();

                    //For each AlgorithmRun I need the container stats
            let container_stats = self.algorithm_run_service.get_all_container_stats(&algo_run).await;

            algo_cs_hashmap.insert(algo, container_stats);

        };

        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned())).unwrap();

            let te_str = test_execution_id.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();


            let full_path = format!("{data_dir_str}/{te_str}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();

            test_cpu_load_line_chart(&algo_cs_hashmap, &full_path);

        };

        Ok(())

    }


}