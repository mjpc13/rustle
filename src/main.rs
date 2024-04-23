use rustle::evo_wrapper::EvoApeArg;
use rustle::evo_wrapper::PlotArg;
use rustle::metrics::ContainerPlot;
use rustle::metrics::Metric;
use rustle::metrics::ContainerStats;
use rustle::task::AdvancedConfig;
use tokio;

use rustle::task::Config;
use rustle::task::Task;

use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    let dataset_path = "/home/mario/Documents/rustle/test/dataset/";
    let params_path = "/home/mario/Documents/rustle/test/config/Point-LIO/";
    //let topics: Vec<String> = vec![
    //   "/lio_odom".to_string(), 
    //];
    //let params_path = "/home/mario/Documents/rustle/test/config/LIO-SAM/";

    let topics: Vec<String> = vec![
        "/Odometry".to_string(), 
     ];

    //let topics: Vec<String> = vec![
    //    "/lio_sam/mapping/odometry".to_string(), 
    //    "/lio_sam/mapping/odometry_incremental".to_string()
    // ];

    let gt_topic: String = String::from("/gt_poses");

    let adv = AdvancedConfig{
        //db_endpoint: String::from("file://test/db"),
        db_database: String::from("point-lio"),
        ..Default::default()
    };

    let test = Config::new(
        "mjpc13/rustle:point-lio".into(), 
        "point-lio".into(), 
        dataset_path.into(), 
        params_path.into(), 
        topics,
        gt_topic, 
        Some(adv) // connect to a custom (and persistent) Surreal database. TODO: Don't pass a database but the endpoint!
        //None
    ).await;
    
    let task1: Task = Task::new(test.unwrap()).await;
    let result = task1.run().await.unwrap();

    let evo_args = EvoApeArg{
        plot: Some(PlotArg::default()),
        ..Default::default()
    };

    let rpe = Metric::compute(
        &result, 
        evo_args, 
        //None
        Some("/home/mario/Documents/rustle/test/results")
    );
    println!("{:?}", rpe);

    ContainerPlot::MemoryUsage.plot(result.stats, "/home/mario/Documents/rustle/test/results");



}
