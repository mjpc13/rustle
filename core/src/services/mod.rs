pub mod error;
pub mod dataset;
pub mod algorithm;
//pub mod test_definition;
pub mod metrics;
pub mod container;
pub mod algorithm_run;
pub mod test_execution;
pub mod iteration;
pub mod stat;

pub mod params;
pub mod tuning;


// Re-export key components
pub use self::{
    error::{ValidationError, AlgorithmError, TestExecutionError, RosError, DbError},
    dataset::DatasetService,
    algorithm::AlgorithmService,
//    test_definition::TestDefinitionService,
    metrics::MetricService,
    container::ContainerService,
    algorithm_run::AlgorithmRunService,
    test_execution::TestExecutionService,
    iteration::IterationService,
    stat::StatService
};