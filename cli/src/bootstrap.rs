use std::sync::Arc;
use bollard::Docker;
use surrealdb::{Surreal, engine::local::RocksDb};
use tokio::sync::Mutex;

use rustle_core::{
    db::*,
    services::*,
    utils::config::Config,
    db::params::ParamsRepo,
    services::params::ParamsService,
    db::tuning::TuningRepo,
    services::tuning::TuningService,
};

pub struct AppContext {
    pub algo_service: AlgorithmService,
    pub dataset_service: DatasetService,
    pub test_exec_service: TestExecutionService,
    pub params_service: ParamsService,
    pub tuning_service: TuningService,
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
    let test_exec_repo = TestExecutionRepo::new(conn_m.clone());
    let odom_repo = OdometryRepo::new(conn_m.clone());
    let algo_run_repo = AlgorithmRunRepo::new(conn_m.clone());
    let stat_repo = StatRepo::new(conn_m.clone());
    let iteration_repo = IterationRepo::new(conn_m.clone());
    let metric_repo = MetricRepo::new(conn_m.clone());
    let params_repo = ParamsRepo::new(conn_m.clone());
    let tuning_repo = TuningRepo::new(conn_m.clone());

    // Services
    let algo_service = AlgorithmService::new(algo_repo, docker.clone());
    let dataset_service = DatasetService::new(dataset_repo);
    let ros_service = RosService::new(odom_repo);
    let stat_service = StatService::new(stat_repo);
    let metric_service = MetricService::new(metric_repo);
    let params_service = ParamsService::new(params_repo, docker.clone());

    let iteration_service = IterationService::new(
        iteration_repo,
        odom_repo,
        docker.clone(),
        dataset_service.clone(),
        stat_service,
        metric_service.clone(),
        params_service.clone(),
    );
    let algo_run_service = AlgorithmRunService::new(algo_run_repo, iteration_service.clone());
    let test_exec_service = TestExecutionService::new(
        test_exec_repo,
        algo_run_service.clone(),
        iteration_service.clone(),
    );

    let tuning_service = TuningService::new(tuning_repo, iteration_service.clone(), algo_run_service, test_exec_service.clone(), dataset_service.clone(), algo_service.clone(), params_service.clone(), metric_service.clone(), docker.clone());

    Ok(AppContext {
        algo_service,
        dataset_service,
        test_exec_service,
        params_service,
        tuning_service,
    })
}