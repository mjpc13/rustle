use clap::Parser;
use db::algorithm;
use serde_yaml::from_reader;
use surrealdb::engine::any::Any;
use std::fs::File;
use surrealdb::engine::local::RocksDb;

use surrealdb::opt::auth::Root;
use surrealdb::Surreal;
use log::{info, error};
use chrono::Utc;

mod models;
mod db;
mod services;

#[derive(Debug, serde::Deserialize)]
struct DatabaseConfig {
    endpoint: String,
    namespace: String,
    database: String
}

#[derive(Debug, serde::Deserialize)]
struct AlgorithmConfig {
    algorithms: Vec<models::Algorithm>,
}

#[derive(Debug, serde::Deserialize)]
struct DatasetConfig {
    datasets: Vec<models::Dataset>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    // Connect to SurrealDB
    let conn = Surreal::new::<RocksDb>("test/db").await?;

    conn.use_ns("rustle")
        .use_db("prod")
        .await?;

    info!("Successfully connected to database");

    // Initialize repositories
    let algo_repo = db::algorithm::AlgorithmRepo::new(conn.clone());
    let dataset_repo = db::dataset::DatasetRepo::new(conn.clone());
    
    let algo_service = services::algorithm::AlgorithmService::new(algo_repo);
    let dataset_service = services::dataset::DatasetService::new(dataset_repo);


    // Process algorithms
    let algorithms_config = "algorithms.yaml";
    let datasets_config = "datasets.yaml";


    let algo_config: AlgorithmConfig = {
        let file = File::open(algorithms_config)?;
        from_reader(file)?
    };

    info!("Found {} algorithms to register", algo_config.algorithms.len());
    for mut algorithm in algo_config.algorithms {
        
        match algo_service.create_algorithm(&mut algorithm).await {
            Ok(_) => info!("Created algorithm: {}", algorithm.name),
            Err(e) => error!("Failed to create algorithm {}: {}", algorithm.name, e),
        }
    }

    // Process datasets
    let dataset_config: DatasetConfig = {
        let file = File::open(datasets_config)?;
        from_reader(file)?
    };

    info!("Found {} datasets to register", dataset_config.datasets.len());
    for mut dataset in dataset_config.datasets {
        dataset.created_at = Utc::now();
        
        match dataset_service.create_dataset(&mut dataset).await {
            Ok(_) => info!("Created dataset: {}", dataset.name),
            Err(e) => error!("Failed to create dataset {}: {}", dataset.name, e),
        }
    }

    info!("Database population completed");
    Ok(())
}