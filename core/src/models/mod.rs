pub mod dataset;
pub mod algorithm;
pub mod test_definitions;
pub mod metrics;
pub mod container;

pub mod ros;
pub mod algorithm_run;
pub mod test_execution;
pub mod iteration;

pub mod messages;

pub mod slam_config;
pub mod parameter_space;
pub mod tuning;

// Re-export main structs for ergonomic imports
pub use self::{
    dataset::Dataset,
    algorithm::Algorithm,
    test_definitions::{TestDefinition, TestType, SimpleTestParams, SpeedTestParams, TestDefinitionsConfig},
    metrics::metric,
    container::Container,
    ros::{Odometry, Header, Pose, Twist},
    algorithm_run::{AlgorithmRun},
    test_execution::{TestExecution, TestExecutionStatus},
    iteration::Iteration,
    messages::ProgressMessage,
    tuning::tuning_config,
};