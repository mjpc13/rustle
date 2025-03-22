use chrono::Utc;

use crate::{
    db::{TestDefinitionRepo, TestExecutionRepo}, models::{test_definition::{TestDefinition, TestType}, test_execution::{TestExecution, TestExecutionStatus}, SimpleTestParams, SpeedTestParams, TestResults}, services::error::ProcessingError
};

pub struct TestExecutionService {
    execution_repo: TestExecutionRepo,
    definition_repo: TestDefinitionRepo
}

impl TestExecutionService {
    pub fn new(execution_repo: TestExecutionRepo, definition_repo: TestDefinitionRepo) -> Self {
        Self { execution_repo, definition_repo }
    }


    //THIS IS MY TASK BATCH!!!
    pub async fn start_execution(
        &self,
        mut execution: TestExecution,
        def: &TestDefinition
    ) -> Result<TestExecution, ProcessingError> {


        let _ = &self.execution_repo.save(&mut execution, def).await;

        // Validate definition exists
        Ok(execution)
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

}