pub mod test_definition;
pub mod simple;
pub mod speed;
pub mod drop;


// Re-export repositories
pub use self::{
    test_definition::{TestDefinition, TestDefinitionsConfig, TestType},
    simple::SimpleTestParams,
    speed::SpeedTestParams,
    drop::DropParams,
};