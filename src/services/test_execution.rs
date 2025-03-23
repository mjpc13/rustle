use chrono::Utc;
use log::{info, warn};

use crate::{
    db::{AlgorithmRunRepo, TestDefinitionRepo, TestExecutionRepo}, models::{algorithm_run::RunAggregates, test_definition::{TestDefinition, TestType}, test_execution::{TestExecution, TestExecutionStatus}, Algorithm, AlgorithmRun, SimpleTestParams, SpeedTestParams, TestResults}, services::error::ProcessingError
};

pub struct TestExecutionService {
    execution_repo: TestExecutionRepo,
    definition_repo: TestDefinitionRepo,
    algorithm_run_repo: AlgorithmRunRepo

}

impl TestExecutionService {
    pub fn new(execution_repo: TestExecutionRepo, definition_repo: TestDefinitionRepo, algorithm_run_repo: AlgorithmRunRepo) -> Self {
        Self { execution_repo, definition_repo, algorithm_run_repo }
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

        // Create algorithm runs based on test type
        match &def.test_type {
            TestType::Simple => self.create_simple_runs(&execution, &list_algos).await?,
            TestType::Speed(params) => (),
        }


        Ok(execution)
    }

    async fn create_simple_runs(
        &self,
        execution: &TestExecution,
        algo_list: &Vec<Algorithm>,
    ) -> Result<(), ProcessingError> {

        for algorithm in algo_list {

            let mut run = AlgorithmRun {
                id: None,
                bag_speed: 1.0,
                aggregates: RunAggregates::default(),
                created_at: Utc::now(),
                duration_secs: 0.0,
            };

            self.algorithm_run_repo.save(
                &mut run,
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