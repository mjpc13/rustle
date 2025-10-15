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
pub enum TestExecutionError {
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

impl From<ValidationError> for TestExecutionError {
    fn from(err: ValidationError) -> Self {
        TestExecutionError::Validation(err.0)
    }
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for ValidationError {}

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

    #[error("List is empty: {0}")]
    Empty(String),

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

    #[error("Error when extracting metrics: {0}")]
    Evo(String),
    
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

#[derive(Debug,Error)]
pub enum PlotError{

    #[error("Unable to plot due to missing data: {0}")]
    MissingData(String),

    #[error("A file named '{0}' already exists at the destination. Use the '--overwrite' flag to replace it.")]
    FileExists(String),

}

#[derive(Debug, Error)]
pub enum TuningError {

    #[error("No field called 'algo_name' was found")]
    NoAlgoName(),

    #[error("No field called 'dataset_name' was found")]
    NoDatasetName(),

    #[error("No field called 'parameters' was found")]
    NoParametersField(),

    #[error("The 'parameters' field is not a json object")]
    NoParametersObject()
}

#[derive(Debug, Error)]
pub enum ParameterSpaceError {

    #[error("There exists no parameter '{0}' for this SLAM algorithm")]
    InvalidKey(String),

    #[error(transparent)]
    BoolParsing(#[from] BoolParsingError),

    #[error(transparent)]
    StringParsing(#[from] StringParsingError),

    #[error(transparent)]
    NumberParsing(#[from] NumberParsingError),

    #[error("The parameter {0} has repeated values, which is not permitted.")]
    RepeatedFields(String),

    #[error("The parameter {0} cannot have higher bound {2} lower than or equal to lower bound {1}.")]
    BoundsProblem(String, String, String),

    #[error("The parameter {0} is out of bounds({1})")]
    OutOfBoundsIndex(String, i32),

    #[error("A parameter of type {0} was asked, but you provided something else")]
    IncompatibleTypes(String),

    #[error("You have repeated values of type {0} in an array, which is not allowed")]
    RepeatedValuesInArray(String),

    #[error("The number array you specified has too many/few values. The correct format is: (first value, step, total number of values)")]
    NumberArrayWrongFormat(),

    #[error("'{0}' is supposed to be an array of {1} values, but value(s) of other type(s) were supplied")]
    WrongTypeInArray(String, String),

    #[error("In a number array, the total number of values cannot be zero")]
    NoValuesInVector(),

    #[error("'{0}' is supposed to be an array of {1} elements, but {2} elements were supplied")]
    WrongArraySize(String, usize, usize),

    #[error("'{0}' is supposed to be an array, but a value of another typed was supplied")]
    NotAnArray(String)

}

#[derive(Debug, Error)]
pub enum BoolParsingError {

    #[error("In boolean Parameter '{0}': the value must be a boolean.")]
    NotABoolean(String),

    #[error("In array for boolean parameter '{0}': the array must not be empty.")]
    EmptyBoolArray(String),

    #[error("In array for boolean parameter '{0}': repeated values are not allowed. At most, the array may contain both 'true' and 'false' values.")]
    RepeatedValues(String),

    #[error("In array for boolean parameter '{0}': the array must not have more than 2 values. At most, it may have 2 values(true and false).")]
    TooManyValuesInArray(String),

    #[error("In array for boolean parameter '{0}': the array contains at least 1 value that is not a boolean.")]
    WrongTypeInArray(String)

}

#[derive(Debug, Error)]
pub enum StringParsingError {

    #[error("In String parameter '{0}': the value must be a String.")]
    NotAString(String),

    #[error("In array for String parameter '{0}': the array must not be empty.")]
    EmptyStringArray(String),

    #[error("In array for String parameter '{0}': the array contains at least 1 value that is not a String.")]
    WrongTypeInArray(String),

    #[error("In array for String parameter '{0}': repeated values are not allowed")]
    RepeatedValues(String)

}

#[derive(Debug, Error)]
pub enum NumberParsingError {

    #[error("In {0} parameter '{1}': the value must be a {0}.")]
    NotANumber(String, String),

    #[error("In array for {0} parameter '{1}': the array must not be empty.")]
    EmptyNumberArray(String, String),

    #[error("In array for numeric parameter '{0}': the supplied array has too many/little values. The correct format is [start, step, number_of_elements].")]
    WrongArrayFormat(String),

    #[error("In array for {0} parameter '{1}': the array contains at least 1 value that is not a {0}.")]
    WrongTypeInarray(String, String),

    #[error("In array for {0} parameter '{1}': the 'start' value must be a {0}.")]
    WrongFirstValue(String, String),

    #[error("In array for {0} parameter '{1}': the 'step' value must be a {0}.")]
    WrongStepType(String, String),

    #[error("In array for {0} parameter '{1}': the 'number_of_elements' value must be a u64.")]
    WrongNumElementsType(String, String)

}