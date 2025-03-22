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

#[derive(Debug, thiserror::Error)]
pub enum TestDefinitionError {
    #[error("Validation error: {0}")]
    Validation(String),
    
    #[error("YAML parsing error: {0}")]
    Yaml(#[from] serde_yaml::Error),
    
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("Database error: {0}")]
    Database(#[from] surrealdb::Error),
}

// ValidationError now wraps String directly
#[derive(Debug)]
pub struct ValidationError(pub String);

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

//Db errors
#[derive(Debug, thiserror::Error)]
pub enum DbError {
    #[error("Database operation failed: {0}")]
    Operation(#[from] surrealdb::Error),
    
    #[error("Record not found: {0}")]
    NotFound(String),
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

}

// Implement conversion from other error types if needed
impl From<surrealdb::Error> for ProcessingError {
    fn from(e: surrealdb::Error) -> Self {
        Self::Database(crate::services::error::DbError::Operation(e))
    }
}