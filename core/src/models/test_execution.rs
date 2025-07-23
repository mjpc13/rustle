use std::{collections::HashMap, fs::File};

use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use surrealdb::sql::Thing;

use crate::{models::{metric::Metric, test_definitions::{CutParams, DropParams}, Algorithm, SpeedTestParams, TestDefinition, TestDefinitionsConfig, TestType}, services::{error::ExecutionError, TestExecutionError, ValidationError}};


#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TestExecutionStatus {
    Scheduled,
    Running,
    Completed,
    Failed(String), // Error message for failure
}
impl ToString for TestExecutionStatus {
    fn to_string(&self) -> String {
        match self {
            TestExecutionStatus::Scheduled => "Scheduled".to_string(),
            TestExecutionStatus::Running => "Running".to_string(),
            TestExecutionStatus::Completed => "Completed".to_string(),
            TestExecutionStatus::Failed(msg) => format!("Failed: {}", msg),
        }
    }
}



#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TestExecution {
    pub id: Option<Thing>,
    pub status: TestExecutionStatus,
    pub created_at: DateTime<Utc>,
    pub start_time: Option<DateTime<Utc>>,
    pub end_time: Option<DateTime<Utc>>,
    pub metrics: HashMap<String, Vec<Metric>>,
    pub def: TestDefinition
}


impl TestExecution {
    pub fn new(def: TestDefinition) -> Self {
        Self {
            id: None,
            status: TestExecutionStatus::Scheduled,
            created_at: Utc::now(),
            start_time: None,
            end_time: None,
            metrics: HashMap::new(),
            def
        }
    }

    // Main entry point that combines loading, validation, and saving
    pub async fn create_from_yaml(
        yaml_path: &str
    ) -> Result<Vec<TestExecution>, TestExecutionError> {
        let definitions = load_and_validate_batch(yaml_path).await?;
        let mut saved = Vec::with_capacity(definitions.len());
        
        for mut def in definitions {
            def.id = None;
            //self.repo.save(&mut def).await?;

            

            saved.push(TestExecution::new(def));
        }
        
        Ok(saved)
    }
}

async fn load_and_validate_batch(
    path: &str
) -> Result<Vec<TestDefinition>, TestExecutionError> {
    let file = File::open(path)?;
    let wrapper: TestDefinitionsConfig = serde_yaml::from_reader(file)?;
    
    let mut definitions = wrapper.test_definitions;
    let now = Utc::now();
    
    for def in &mut definitions {
        // Set timestamps if missing
        def.updated_at = now;
        
        // Validate test type parameters
        match &def.test_type {
            TestType::Simple => validate(def)?,
            TestType::Speed(params) => validate_speed(&params)?,
            TestType::Drop(params) => validate_drop(params)?,
            TestType::Cut(params) => validate_cut(params)?
        }
    }
    Ok(definitions)
}

fn validate(def: &TestDefinition) -> Result<(), ValidationError> {
    if def.iterations == 0 {
        Err(ValidationError("Iterations must be > 0".into()))
    } else {
        Ok(())
    }
}

fn validate_speed(params: &SpeedTestParams) -> Result<(), ValidationError> {
    if params.speed_factors.is_empty() {
        return Err(ValidationError("Speed factors cannot be empty".into()));
    }
    if params.speed_range<= params.speed_step{
        return Err(ValidationError("Speed step cannot be greater than range".into()));
    }
    Ok(())
}

fn validate_drop(_params: &DropParams) -> Result<(), ValidationError> {
    let todo = true; //TODO:Check what I should validate in these params!
    Ok(())
}

fn validate_cut(_params: &CutParams) -> Result<(), ValidationError> {
    let todo = true; //TODO:Check what I should validate in these params!
    Ok(())
}