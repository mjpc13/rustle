use thiserror::Error;

#[derive(Debug, Error)]
pub enum AlgorithmError {
    #[error("Validation error: {0}")]
    Validation(String),
    
    #[error("Database error: {0}")]
    Database(#[from] surrealdb::Error),
    
    #[error("YAML parsing error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

#[derive(Debug, Error)]
pub enum TestDefinitionError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("YAML error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    
    #[error("Validation error: {0}")]
    Validation(String),
    
    #[error("Database error: {0}")]
    Database(#[from] surrealdb::Error),
}

#[derive(Debug)]
pub struct ValidationError(pub String);

impl From<ValidationError> for TestDefinitionError {
    fn from(err: ValidationError) -> Self {
        TestDefinitionError::Validation(err.0)
    }
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for ValidationError {}



//Metric Errors
#[derive(Debug, thiserror::Error)]
pub enum MetricError {
    #[error("Validation error: {0}")]
    Validation(String),
    
    #[error("Database error: {0}")]
    Database(#[from] surrealdb::Error),
}

//ROS errors
#[derive(Debug, thiserror::Error)]
pub enum RosProcessingError {
    #[error("Storage error: {0}")]
    Storage(String),
    
    #[error("Query error: {0}")]
    Query(String),
    
    //#[error("Conversion error: {0}")]
    //Conversion(#[from] crate::models::ros::RosError),
}


#[derive(Debug, thiserror::Error)]
pub enum DbError {
    #[error("Database operation failed: {0}")]
    Operation(#[from] surrealdb::Error),
    
    #[error("Record not found: {0}")]
    NotFound(String),
    
    #[error("Missing required field: {0}")]
    MissingField(&'static str),
    
    #[error("Validation failed: {0}")]
    Validation(String),
}

#[derive(Debug, Error)]
pub enum ProcessingError {
    #[error("Database error: {0}")]
    Database(#[from] crate::services::error::DbError),

    #[error("Resource conflict: {0}")]
    Conflict(String),
    
    #[error("Processing failure: {0}")]
    General(String),
    
    #[error("Invalid iteration data: {0}")]
    InvalidIteration(String),
    
    #[error("Algorithm run not found: {0}")]
    NotFound(String),

    #[error("Parameter validation failed: {0}")]
    Validation(String),
    
    #[error("Test definition mismatch: {0}")]
    DefinitionMismatch(String),

    #[error("Docker image not found: {0}")]
    ImageNotFound(String),
    
    #[error("Docker operation failed: {0}")]
    DockerOperation(String),

}

// Implement conversion from other error types if needed
impl From<surrealdb::Error> for ProcessingError {
    fn from(e: surrealdb::Error) -> Self {
        Self::Database(crate::services::error::DbError::Operation(e))
    }
}