pub mod test_definition;
pub mod simple;
pub mod speed;
pub mod degradation;


// Re-export repositories
pub use self::{
    test_definition::{TestDefinition, TestDefinitionsConfig, TestType},
    simple::SimpleTestParams,
    speed::SpeedTestParams,
    degradation::SensorDegradationParams,
};