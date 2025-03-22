pub mod error;
pub mod dataset;
pub mod algorithm;
pub mod test_definition;
pub mod metrics;
pub mod container;
pub mod odometry;
pub mod algorithm_run;
pub mod test_execution;


// Re-export key components
pub use self::{
    error::{ValidationError, AlgorithmError, TestDefinitionError, MetricError, RosProcessingError, DbError},
    dataset::DatasetService,
    algorithm::AlgorithmService,
    test_definition::TestDefinitionService,
    metrics::MetricService,
    container::ContainerService,
    odometry::OdometryService,
    algorithm_run::AlgorithmRunService,
    test_execution::TestExecutionService
};