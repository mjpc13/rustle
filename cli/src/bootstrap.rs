use std::sync::Arc;
use bollard::Docker;
use surrealdb::{Surreal, engine::local::RocksDb};
use tokio::sync::Mutex;

use rustle_core::{
    db::*,
    services::*,
    utils::config::Config,
};

pub struct AppContext {
    pub algo_service: AlgorithmService,
    pub dataset_service: DatasetService,
    pub test_def_service: TestDefinitionService,
    pub test_exec_service: TestExecutionService,
    pub config: Config,
}

pub async fn build_app() -> Result<AppContext, Box<dyn std::error::Error>> {
    let config = Config::load()?;

    let conn = Surreal::new::<RocksDb>(&config.database.path).await?;
    conn.use_ns(&config.database.namespace)
        .use_db(&config.database.name)
        .await?;
    let conn_m = Arc::new(Mutex::new(conn));

    let docker = Arc::new(Docker::connect_with_local_defaults()?);

    // Repositories
    let algo_repo = AlgorithmRepo::new(conn_m.clone());
    let dataset_repo = DatasetRepo::new(conn_m.clone());
    let test_def_repo = TestDefinitionRepo::new(conn_m.clone());
    let test_exec_repo = TestExecutionRepo::new(conn_m.clone());
    let odom_repo = OdometryRepo::new(conn_m.clone());
    let algo_run_repo = AlgorithmRunRepo::new(conn_m.clone());
    let stat_repo = StatRepo::new(conn_m.clone());
    let iteration_repo = IterationRepo::new(conn_m.clone());
    let metric_repo = MetricRepo::new(conn_m.clone());

    // Services
    let algo_service = AlgorithmService::new(algo_repo, docker.clone());
    let dataset_service = DatasetService::new(dataset_repo);
    let test_def_service = TestDefinitionService::new(test_def_repo.clone());
    let ros_service = RosService::new(odom_repo);
    let stat_service = StatService::new(stat_repo);
    let metric_service = MetricService::new(metric_repo);
    let iteration_service = IterationService::new(
        iteration_repo,
        docker.clone(),
        ros_service,
        dataset_service.clone(),
        stat_service,
        metric_service,
    );
    let algo_run_service = AlgorithmRunService::new(algo_run_repo, iteration_service.clone());
    let test_exec_service = TestExecutionService::new(
        test_exec_repo,
        test_def_repo,
        algo_run_service,
        iteration_service,
    );

    Ok(AppContext {
        algo_service,
        dataset_service,
        test_def_service,
        test_exec_service,
        config,
    })
}