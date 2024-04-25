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
    let gt_topic: String = String::from("/gt_poses");
    
    
    // iG-LIO //
    
    //let params_file = "/home/mario/Documents/rustle/test/config/ig-lio/params.yaml";
    //let image = "mjpc13/rustle:ig-lio";
    //let topics: Vec<String> = vec![
    //   "/lio_odom".to_string(), 
    //];
    //let slam_method = "ig-lio";

    //=============================//


    // LIO-SAM 6 axis Cant compile the Dockerfile of this algorithm//
    
    //let params_file = "/home/mario/Documents/rustle/test/config/lio-sam-6axis/params.yaml";
    //let image = "mjpc13/rustle:lio-sam-6axis";
    //let topics: Vec<String> = vec![
    //    "/lio_sam/mapping/odometry".to_string(), 
    //    "/lio_sam/mapping/odometry_incremental".to_string()
    // ];
    //let slam_method = "lio-sam-6axis";

    //===============================//


    // FAST-LIO //

    let params_file = "/home/mario/Documents/rustle/test/config/fast-lio/params.yaml";
    let image = "mjpc13/rustle:fast-lio";
    let topics: Vec<String> = vec![
        "/Odometry".to_string(), 
    ];
    let slam_method = "fast-lio";

    //===============================//


    //Optional Advanced Configuration object
    let adv = AdvancedConfig{
        //db_endpoint: String::from("file://test/db"),
        db_database: String::from(slam_method),
        ..Default::default()
    };

    let config = Config::new(
        image.into(), 
        slam_method.into(), 
        dataset_path.into(), 
        params_file.into(), 
        topics,
        gt_topic, 
        Some(adv)
        //None
    ).await.unwrap();
    
    //Creates a new task
    let task: Task = Task::new(config).await;

    //Runs the task (the SLAM algorithm) and returns list of Odometry
    let result = task.run().await.unwrap();

    //EVALUATE RESULTS Part//

    //Evo Arg Parameters
    let evo_args = EvoApeArg{
        plot: Some(PlotArg::default()),
        ..Default::default()
    };

    let ape = Metric::compute(
        &result, 
        evo_args, 
        None
        //Some("/home/mario/Documents/rustle/test/results")
    );

    //Plots for CPU Load and Memory Usage
    ContainerPlot::MemoryUsage.plot(&result.stats, "/home/mario/Documents/rustle/test/results/memory_usage.png");
    ContainerPlot::MemoryUsagePerSec.plot(&result.stats, "/home/mario/Documents/rustle/test/results/memory_usage_per_sec.png");
    ContainerPlot::LoadPercentage.plot(&result.stats, "/home/mario/Documents/rustle/test/results/load_percentage.png");

}
