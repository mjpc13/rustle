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
//#[derive(Debug, thiserror::Error)]
//pub enum MetricError {
//    #[error("Validation error: {0}")]
//    Validation(String),
//    
//    #[error("Database error: {0}")]
//    Database(#[from] surrealdb::Error),
//}

//ROS errors
#[derive(Debug, thiserror::Error)]
pub enum RosError {
    #[error("Storage error: {0}")]
    Storage(String),
    
    #[error("Query error: {0}")]
    Query(String),

    #[error("Parse error: failed to convert from {from} to {to}")]
    ParseError { from: String, to: String },
    
    #[error("Missing header for ROS type {rostype}")]
    MissingHeader { rostype: String },
    
    #[error("Database error: {0}")]
    DatabaseError(#[from] surrealdb::Error),
    
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("Format error: {0}")]
    FormatError(String),
}


#[derive(Debug, Error)]
pub enum DbError {
    #[error("Database operation failed: {0}")]
    Operation(#[from] surrealdb::Error),
    
    #[error("Record not found: {0}")]
    NotFound(String),
    
    #[error("Missing required field: {0}")]
    MissingField(&'static str),
    
    #[error("Validation failed: {0}")]
    Validation(String),

    #[error("Invalid Data: {0}")]
    InvalidData(&'static str),

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

    #[error("Missing required field: {0}")]
    MissingField(String),

}

// Implement conversion from other error types if needed
impl From<surrealdb::Error> for ProcessingError {
    fn from(e: surrealdb::Error) -> Self {
        Self::Database(crate::services::error::DbError::Operation(e))
    }
}

// RUN errors
#[derive(Debug, Error)]
pub enum RunError {
    
    #[error("Docker operation failed: {0}")]
    Docker(#[from] bollard::errors::Error),
    
    #[error("Execution error: {0}")]
    Execution(String),
    
    #[error("Task failed to complete: {0}")]
    Join(#[from] tokio::task::JoinError),
}

#[derive(Debug,Error)]
pub enum EvoError{

    #[error("Could not run evo tool: {0}")]
    MissingEvo(String),

    #[error("Evo command error: {0}")]
    CommandError(String),

}

#[derive(Debug,Error)]
pub enum MetricError{

    #[error("Unable to compute metric: {0}")]
    ComputeError(String),

    #[error("Metric field is empty: {0}")]
    MissingError(String),

    #[error("Unable to read metric file: {0}")]
    IOError(String),

    #[error("Failed to cast input as a float: {0}")]
    ParseError(String)

}

#[derive(Debug,Error)]
pub enum ExecutionError{

    #[error("Unable to compute test execution: {0}")]
    ComputeError(String),


}