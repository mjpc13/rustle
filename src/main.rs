
use std::result;

use rustle::evo_wrapper::EvoApeArg;
use rustle::evo_wrapper::PlotArg;
use rustle::metrics::ContainerPlot;
use rustle::metrics::Metric;
use rustle::metrics::ContainerStats;
use rustle::task::AdvancedConfig;
use rustle::task::TaskOutput;
use tokio;

use rustle::task::Config;
use rustle::task::Task;

use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    //Dataset related
    let dataset_path = "/home/mario/Documents/rustle/test/dataset/";
    let gt_topic: &str = "/gt_poses";
    let adv = AdvancedConfig{
        //db_endpoint: String::from("file://test/db"),
        //db_database: String::from(ig_lio_name),
        ..Default::default()
    };

    //Put algorithms I want to run in a Vector
    let mut configs: Vec<Config> = Vec::new();
    
    // iG-LIO //
    configs.push(
        Config::new(
            "mjpc13/rustle:ig-lio", //docker image name
            "ig-lio", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/ig-lio/params.yaml",  //Yaml config file for the algorithm 
            vec!["/lio_odom"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            Some(&adv) //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    //=============================//

    // LIO-SAM-6AXIS //
    configs.push(
        Config::new(
            "mjpc13/rustle:lio-sam-6axis", //docker image name
            "lio-sam-6axis", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/lio-sam-6axis/params.yaml",  //Yaml config file for the algorithm 
            vec!["/lio_sam_6axis/mapping/odometry_incremental"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            Some(&adv) //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );
    
    //let topics: Vec<String> = vec![
    //   "/lio_sam_6axis/mapping/odometry".to_string(), 
    //   "/lio_sam_6axis/mapping/odometry_incremental".to_string(), 
    //];

    //=============================//

    // FAST-LIO //

    configs.push(
        Config::new(
            "mjpc13/rustle:fast-lio",
            "fast-lio",
            dataset_path,
            "/home/mario/Documents/rustle/test/config/fast-lio/params.yaml",
            vec!["/Odometry"],
            gt_topic,
            None
        ).await.unwrap()
    );

    //===============================//

    // POINT-LIO //

    configs.push(
        Config::new(
            "mjpc13/rustle:point-lio", //docker image name
            "point-lio", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/point-lio/params.yaml",  //Yaml config file for the algorithm 
            vec!["/aft_mapped_to_init"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    // LVI-SAM //
    //configs.push(Config::new(
    //        "mjpc13/rustle:lvi-sam", //docker image name
    //        "lvi-sam", //algorithm name
    //        dataset_path, // path to the directory of the rosbag
    //        "/home/mario/Documents/rustle/test/config/lvi-sam/camera_params.yaml",  //Yaml config file for the algorithm 
    //        vec!["/Odometry"], //Vector of topics to record
    //        gt_topic, //topic of the ground truth
    //        None //Advanced arguments (database and docker stuff) replace with None for default config
    //    ).await.unwrap()
    //);

    //===============================//
    
    //Creates a new task
    //let results = configs
    //    .into_iter()
    //    .map(|c| async  {
    //        let task: Task = Task::new(c).await;
    //        let res = task.run().await.unwrap();
    //    });
    //let results = futures_util::future::join_all(results).await;

    let mut results: Vec<TaskOutput> = Vec::<TaskOutput>::new();

    for c in configs{
        let task = Task::new(c.clone()).await;
        let res = task.run().await.unwrap();
        results.push(res);

    }


    //let lvi_sam_task: Task = Task::new(lvi_sam_config).await;

    //EVALUATE RESULTS//
    
    let mut evo_md = String::from("| Name | Max | Median | Min | RMSE | SSE | Std |\n|--------|-------|--------|-------|-------|-------|-------|\n");
    let names = vec!["ig-lio", "lio-sam", "fast-lio", "point-lio"];

    for (r, n) in results.iter().zip(names.iter()){
        let evo_res = Metric::compute(
            r,
            EvoApeArg{
                plot: Some(PlotArg::default()),
                ..Default::default()
            },
            None
        ).unwrap();

        evo_md = evo_md + &evo_res[0].to_md(n);

    }
    println!("{}", evo_md);





    //Plots for CPU Load and Memory Usage
    ContainerPlot::MemoryUsage.plot(&results, "/home/mario/Documents/rustle/test/results/memory_usage.svg");
    ContainerPlot::LoadPercentage.plot(&results, "/home/mario/Documents/rustle/test/results/load_percentage.svg");
    ContainerPlot::MemoryUsagePerSec.plot(&results, "/home/mario/Documents/rustle/test/results/memory_usage_per_sec.svg");

    //ContainerPlot::MemoryUsagePerSec.plot(&vec![&ig_lio_result, &fast_lio_result, &point_lio_result], "/home/mario/Documents/rustle/test/results/memory_usage_per_sec.svg");
    //ContainerPlot::MemoryUsage.plot(&vec![&ig_lio_result, &fast_lio_result, &point_lio_result], "/home/mario/Documents/rustle/test/results/memory_usage.svg");
    //ContainerPlot::LoadPercentage.plot(&vec![&ig_lio_result, &fast_lio_result, &point_lio_result], "/home/mario/Documents/rustle/test/results/load_percentage.svg");

}