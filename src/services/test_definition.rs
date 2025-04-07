use std::fs::File;

use chrono::Utc;

use crate::{db::TestDefinitionRepo, models::{test_definitions::test_definition::{TestDefinitionsConfig}, SpeedTestParams, TestDefinition, TestType}};
use super::{error::ValidationError, TestDefinitionError};

pub struct TestDefinitionService {
    repo: TestDefinitionRepo,
}


impl TestDefinitionService {

    pub fn new(repo: TestDefinitionRepo) -> Self {
        Self { repo }
    }

    // Main entry point that combines loading, validation, and saving
    pub async fn create_from_yaml(
        &self,
        yaml_path: &str
    ) -> Result<Vec<TestDefinition>, TestDefinitionError> {
        let definitions = self.load_and_validate_batch(yaml_path).await?;
        let mut saved = Vec::with_capacity(definitions.len());
        
        for mut def in definitions {
            def.id = None;
            self.repo.save(&mut def).await?;
            saved.push(def);
        }
        
        Ok(saved)
    }

    async fn load_and_validate_batch(
        &self,
        path: &str
    ) -> Result<Vec<TestDefinition>, TestDefinitionError> {
        let file = File::open(path)?;
        let wrapper: TestDefinitionsConfig = serde_yaml::from_reader(file)?;
        
        let mut definitions = wrapper.test_definitions;
        let now = Utc::now();
        
        for def in &mut definitions {
            // Set timestamps if missing

            def.updated_at = now;
            
            // Validate test type parameters
            match &def.test_type {
                TestType::Simple => self.validate(def)?,
                TestType::Speed(params) => self.validate_speed(&params)?,
            }
        }
        
        Ok(definitions)
    }


    fn validate_definition(
        &self,
        def: &TestDefinition
    ) -> Result<(), ValidationError> {
        match &def.test_type {
            TestType::Simple => self.validate(def),
            TestType::Speed(params) => {
                let _ = self.validate(def)?;
                self.validate_speed(params)
            },
            // TestType::Drop(params) => self.validate_drop(params),
        }
    }

    fn validate(&self, def: &TestDefinition) -> Result<(), ValidationError> {
        if def.iterations == 0 {
            Err(ValidationError("Iterations must be > 0".into()))
        } else {
            Ok(())
        }
    }

    fn validate_speed(&self, params: &SpeedTestParams) -> Result<(), ValidationError> {
        if params.speed_factors.is_empty() {
            return Err(ValidationError("Speed factors cannot be empty".into()));
        }
        Ok(())
    }

    pub async fn get_all(&self) -> Vec<TestDefinition>{
        self.repo.list().await.unwrap()
    }

    //fn validate_drop(&self, params: &DropTestParams) -> Result<(), ValidationError> {
    //    if params.topics.is_empty() {
    //        return Err(ValidationError("Drop test requires at least one topic".into()));
    //    }
    //    Ok(())
    //}
}