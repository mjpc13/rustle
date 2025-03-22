use chrono::Utc;

use crate::{
    db::{TestDefinitionRepo, TestExecutionRepo}, models::{test_definition::{TestDefinition, TestType}, test_execution::{TestExecution, TestExecutionStatus}, SimpleTestParams, SpeedTestParams, TestResults}, services::error::ProcessingError
};

pub struct TestExecutionService {
    execution_repo: TestExecutionRepo,
    definition_repo: TestDefinitionRepo,
}

impl TestExecutionService {
    pub fn new(execution_repo: TestExecutionRepo, definition_repo: TestDefinitionRepo) -> Self {
        Self { execution_repo, definition_repo }
    }

    pub async fn start_execution(
        &self,
        mut execution: TestExecution
    ) -> Result<TestExecution, ProcessingError> {
        let definition = self.definition_repo.get(&execution.test_definition_id)
            .await?
            .ok_or(ProcessingError::NotFound("Test definition".into()))?;

        self.validate_parameters(&execution.parameters, &definition)?;
        
        execution.status = TestExecutionStatus::Running;
        execution.start_time = Some(Utc::now());
        
        //self.execution_repo.save(&execution).await?;
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

    fn validate_parameters(
        &self,
        params: &serde_json::Value,
        definition: &TestDefinition
    ) -> Result<(), ProcessingError> {
        match &definition.test_type {
            TestType::Simple(expected_params) => {
                // Validate against SimpleTestParams structure
                let received_params: SimpleTestParams = serde_json::from_value(params.clone())
                    .map_err(|e| ProcessingError::Validation(
                        format!("Invalid simple test parameters: {}", e)
                    ))?;
                
                // Example validation: Ensure iterations match definition
                if received_params.iterations != expected_params.iterations {
                    return Err(ProcessingError::Validation(
                        "Iteration count mismatch with test definition".into()
                    ));
                }
                
                Ok(())
            },
            TestType::Speed(expected_params) => {
                // Validate against SpeedTestParams structure
                let received_params: SpeedTestParams = serde_json::from_value(params.clone())
                    .map_err(|e| ProcessingError::Validation(
                        format!("Invalid speed test parameters: {}", e)
                    ))?;
    
                // Example validation: Check speed factors
                if received_params.speed_factors != expected_params.speed_factors {
                    return Err(ProcessingError::Validation(
                        "Speed factors mismatch with test definition".into()
                    ));
                }
                
                Ok(())
            },
            // Add Drop variant when implemented
            _ => Err(ProcessingError::Validation(
                "Unsupported test type".into()
            ))
        }
    }
}

// Validation helpers
fn validate_simple_params(params: &serde_json::Value) -> Result<(), ProcessingError> {
    if params.get("iterations").is_none() {
        return Err(ProcessingError::Validation(
            "Simple test requires iterations parameter".into()
        ));
    }
    Ok(())
}