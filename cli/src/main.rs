mod args;
mod handlers;
mod bootstrap;

use args::*;
use bootstrap::*;
use handlers::{handle_algo, handle_config, handle_dataset, handle_test};
use clap::Parser;
use env_logger;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let args = RustleArgs::parse();
    let app = build_app().await?;

    match args.command {
        CommandType::Dataset(ds_cmd) => handle_dataset(ds_cmd, &app.dataset_service).await?,
        CommandType::Algo(algo_cmd) => handle_algo(algo_cmd, &app.algo_service).await?,
        CommandType::Test(test_cmd) => handle_test(test_cmd, &app.test_def_service, &app.test_exec_service).await?,
        CommandType::Config(config_cmd) => handle_config(config_cmd).await?,
    }

    Ok(())
}