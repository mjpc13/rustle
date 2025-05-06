pub mod dataset;
pub mod algorithm;
pub mod test;
pub mod config;

pub use dataset::handle_dataset;
pub use algorithm::handle_algo;
pub use test::handle_test;
pub use config::handle_config;