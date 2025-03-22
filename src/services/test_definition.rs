use chrono::Utc;

use crate::{db::TestDefinitionRepo, models::{SimpleTestParams, SpeedTestParams, TestDefinition, TestType}};
use super::{error::ValidationError, TestDefinitionError};

pub struct TestDefinitionService {
    repo: TestDefinitionRepo,
}


impl TestDefinitionService {
    async fn load_and_validate_yaml(
        &self,
        path: &str
    ) -> Result<TestDefinition, TestDefinitionError> {
        let file = std::fs::File::open(path)
            .map_err(|e| TestDefinitionError::Io(e))?;
        
        let mut def: TestDefinition = serde_yaml::from_reader(file)
            .map_err(|e| TestDefinitionError::Yaml(e))?;
        
        // Validate based on test type
        match &def.test_type {
            TestType::Simple(params) => self.validate_simple(params),
            TestType::Speed(params) => self.validate_speed(params),
            //TestType::Drop(params) => self.validate_drop(params),
        }
        .map_err(|e| TestDefinitionError::Validation(e.0))?;

        def.updated_at = Utc::now();
        Ok(def)
    }

    fn validate_simple(&self, params: &SimpleTestParams) -> Result<(), ValidationError> {
        if params.iterations == 0 {
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

    //fn validate_drop(&self, params: &DropTestParams) -> Result<(), ValidationError> {
    //    if params.topics.is_empty() {
    //        return Err(ValidationError("Drop test requires at least one topic".into()));
    //    }
    //    Ok(())
    //}
}