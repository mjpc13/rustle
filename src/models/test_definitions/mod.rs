pub mod test_definition;
pub mod simple;
pub mod speed;
pub mod drop;
pub mod cut;


// Re-export repositories
pub use self::{
    test_definition::{TestDefinition, TestDefinitionsConfig, TestType},
    simple::SimpleTestParams,
    speed::SpeedTestParams,
    drop::DropParams,
    cut::{Cut, CutParams}
};