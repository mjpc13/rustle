#![allow(unused)]

//! # RustLE - A Rust Unique Simple Testbench for Localization Experiments
//! 
//! Rustle is a simple API that allows to run and process automatically multiple SLAM/localization algorithms in Rust.
//! It takes three arguments:
//! 
//! * A Docker image with a SLAM algorithm (e.g LIO-SAM);
//! * A ROS dataset (e.g BotanicGarden);
//! * A YAML config file to set the parameters for the algorithm to run.
//! 
//! Rustle outputs:
//! * The APE/RTE metrics alongside relevant graphics (this is based on the evo tool)
//! * The computational load of the CPU and relevant graphics
//! 


pub mod config;
pub mod models;
pub mod db;
pub mod services;
pub mod utils;