use db::{test_definition, TestDefinitionRepo};
use models::{TestDefinitionsConfig, TestExecutionStatus};
use serde_yaml::from_reader;
use services::{AlgorithmService, DatasetService, TestDefinitionService, TestExecutionService};
use tokio::sync::Mutex;
use std::{fs::File, sync::Arc};
use surrealdb::engine::local::RocksDb;

use surrealdb::Surreal;
use log::{info, error};
use chrono::Utc;

mod models;
mod db;
mod services;

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

    // Connect to SurrealDB This probably will have to be inside a Arc<Mutex>
    let conn = Surreal::new::<RocksDb>("test/db").await?;

    let conn_m = Arc::new(Mutex::new(conn));


    conn_m.lock().await.use_ns("rustle")
        .use_db("prod")
        .await?;

    info!("Successfully connected to database");

    // Initialize repositories
    let algo_repo = db::algorithm::AlgorithmRepo::new(conn_m.clone());
    let dataset_repo = db::dataset::DatasetRepo::new(conn_m.clone());
    let test_def_repo = db::test_definition::TestDefinitionRepo::new(conn_m.clone());
    let test_exec_repo = db::test_execution::TestExecutionRepo::new(conn_m.clone());

    // Initialize services
    let algo_service = AlgorithmService::new(algo_repo);
    let dataset_service = DatasetService::new(dataset_repo);
    let test_def_service = TestDefinitionService::new(test_def_repo.clone());
    let test_exec_service = TestExecutionService::new(test_exec_repo, test_def_repo.clone());


    // Load and process algorithms
    let algo_config: AlgorithmConfig = load_yaml_config("algorithms.yaml")?;
    info!("Found {} algorithms to register", algo_config.algorithms.len());
    process_algorithms(&algo_service, algo_config).await;

    // Load and process datasets
    let dataset_config: DatasetConfig = load_yaml_config("datasets.yaml")?;
    info!("Found {} datasets to register", dataset_config.datasets.len());
    process_datasets(&dataset_service, dataset_config).await;

    // Load and process test definitions
    let test_def_config: TestDefinitionsConfig = load_yaml_config("tests.yaml")?;
    info!("Processing test definitions");
    process_test_definitions(&test_def_service, test_def_config).await;

    // Execute all test definitions
    info!("Starting test executions");
    execute_all_tests(&test_exec_service, &test_def_service).await;


    
    Ok(())
}



fn load_yaml_config<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    Ok(from_reader(file)?)
}

async fn process_algorithms(service: &AlgorithmService, config: AlgorithmConfig) {
    for mut algorithm in config.algorithms {
        match service.create_algorithm(&mut algorithm).await {
            Ok(_) => info!("Created algorithm: {}", algorithm.name),
            Err(e) => error!("Failed to create algorithm {}: {}", algorithm.name, e),
        }
    }
}

async fn process_datasets(service: &DatasetService, config: DatasetConfig) {
    for mut dataset in config.datasets {
        dataset.created_at = Utc::now();
        match service.create_dataset(&mut dataset).await {
            Ok(_) => info!("Created dataset: {}", dataset.name),
            Err(e) => error!("Failed to create dataset {}: {}", dataset.name, e),
        }
    }
}

async fn process_test_definitions(service: &TestDefinitionService, config: TestDefinitionsConfig) {
    match service.create_from_yaml("tests.yaml").await {
        Ok(definitions) => {
            info!("Created {} test definitions", definitions.len());
            for def in definitions {
                info!("- {} ({:?})", def.name, def.id.unwrap());
            }
        }
        Err(e) => error!("Failed to create test definitions: {}", e),
    }
}

async fn execute_all_tests(
    test_exec_service: &TestExecutionService,
    test_def_service: &TestDefinitionService,
) {
    // Get all test definitionsk
    let definitions = test_def_service.get_all().await;

    for def in definitions {
        info!("Processing test definition: {}", def.name);
        
        // Create initial execution object
        let mut execution = models::TestExecution {
            id: None,
            status: TestExecutionStatus::Scheduled,
            start_time: None,
            end_time: None,
            results: None,
        };

        // Start execution
        let running_exec = match test_exec_service.start_execution(execution, &def).await {
            Ok(exec) => {
                info!("Started execution for {} ({})", def.name, exec.id.as_ref().unwrap());
                exec
            }
            Err(e) => {
                error!("Failed to start execution for {}: {}", def.name, e);
                continue;
            }
        };

    }
}