use std::sync::Arc;

use chrono::Utc;
use log::{debug, info, warn};
use tokio::sync::Mutex;

use crate::{
    db::{TestDefinitionRepo, TestExecutionRepo}, 
    models::{algorithm_run::RunAggregates, test_definition::{TestDefinition, TestType}, test_execution::{TestExecution, TestExecutionStatus}, Algorithm, AlgorithmRun, Iteration, SimpleTestParams, SpeedTestParams, TestResults}, services::error::ProcessingError
};

use super::{AlgorithmRunService, DatasetService, IterationService};

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

        // Validate definition exists
        //Implement the TaskBatch Logic!

        let list_algos = self.execution_repo.get_algos(&execution).await?;

        // Create algorithm runs and their iterations and based on test type
        match &def.test_type {
            TestType::Simple => self.create_simple_runs(&execution, &list_algos).await?,
            TestType::Speed(params) => (), // TODO: need to do this...
        }

        let mut list_iterations = self.execution_repo
            .get_iterations(
                execution.id.as_ref()  // Get reference to inner Thing
                    .ok_or(ProcessingError::General("TestExecution ID".into()))?
            ).await?;

        //debug!("List of iterations: {:?}", list_iterations);

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
            info!("This should only run after all the things ran")
        }


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
                &algorithm.id.clone().unwrap()
            ).await?;

        }
        Ok(())
    }




    //async fn create_speedbag_runs(
    //    &self,
    //    execution: &TestExecution,
    //    definition: &TestDefinition,
    //    params: SpeedTestParams,
    //) -> Result<(), ProcessingError> {
    //    // Implementation for speed bag tests
    //    // Create multiple runs with different parameters
    //    // Example: 1 run per speed setting
    //    for speed_setting in params.speed_levels {
    //        for algorithm_name in &definition.algo_list {
    //            // Similar to simple runs but with speed parameters
    //            let mut run = AlgorithmRun {
    //                id: None,
    //                test_execution_id: execution.id.as_ref().unwrap().clone(),
    //                algorithm_id: algorithm.id.unwrap().clone(),
    //                aggregates: RunAggregates::default(),
    //                created_at: Utc::now(),
    //                duration_secs: 0.0,
    //                speed_setting: Some(speed_setting), // Add this field if needed
    //            };
    //            // Save run
    //        }
    //    }
    //    Ok(())
    //}







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

}