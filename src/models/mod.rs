pub mod dataset;
pub mod algorithm;
pub mod test_definition;
pub mod metric;
pub mod container;

pub mod ros;
pub mod algorithm_run;
pub mod test_execution;
pub mod iteration;

// Re-export main structs for ergonomic imports
pub use self::{
    dataset::Dataset,
    algorithm::Algorithm,
    test_definition::{TestDefinition, TestType, SimpleTestParams, SpeedTestParams, TestDefinitionsConfig},
    metric::{
        Metric,
        MetricType,
        StatisticalMetrics,
        SingleValueMetric
    },
    container::Container,
    ros::{Odometry, Header, Pose, Twist},
    algorithm_run::{AlgorithmRun},
    test_execution::{TestExecution, TestExecutionStatus, Environment, TestResults},
    iteration::Iteration
};

