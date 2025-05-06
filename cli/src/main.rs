
mod args;
mod handlers;

use args::*;
use handlers::{handle_algo, handle_config, handle_dataset, handle_test};

use clap::Parser;
use log::info;
use rustle_core::{services::*, models::*, db::*}; // you may want to re-export services/models in core/lib.rs
use rustle_core::config::Config;
use std::fs::File;
use std::sync::Arc;
use tokio::sync::Mutex;
use surrealdb::Surreal;
use surrealdb::engine::local::RocksDb;


#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let args = RustleArgs::parse();

    // Setup DB and Docker
    //This should be .local/share/

    let config = Config::load()?;
    let db_path = &config.database.path;
    let db_ns = &config.database.namespace;
    let db_name = &config.database.name;


    // Connect to SurrealDB using config path
    let conn = Surreal::new::<RocksDb>(db_path).await?;
    conn.use_ns(db_ns).use_db(db_name).await?;
    let conn_m = Arc::new(Mutex::new(conn));

    // Create repositories and services
    let dataset_repo = DatasetRepo::new(conn_m.clone());
    let algo_repo = AlgorithmRepo::new(conn_m.clone());
    let test_def_repo = TestDefinitionRepo::new(conn_m.clone());

    let dataset_service = DatasetService::new(dataset_repo);
    let algo_service = AlgorithmService::new(algo_repo, Arc::new(bollard::Docker::connect_with_local_defaults()?));
    let test_def_service = TestDefinitionService::new(test_def_repo);

    // Match CLI commands
    match args.command {
        CommandType::Dataset(ds_cmd) => handle_dataset(ds_cmd, &dataset_service).await?,
        CommandType::Algo(algo_cmd) => handle_algo(algo_cmd, &algo_service).await?,
        CommandType::Test(test_cmd) => handle_test(test_cmd, &test_def_service).await?,
        CommandType::Config(config_cmd) => handle_config(config_cmd).await?,
    }

    Ok(())
}