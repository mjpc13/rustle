use std::sync::Arc;

use bollard::Docker;
use rustle::db::load_from_file;
use rustle::evo_wrapper::EvoApeArg;
use rustle::evo_wrapper::PlotArg;
use rustle::metrics::ContainerPlot;
use rustle::metrics::Metric;
use rustle::config::AdvancedConfig;
use rustle::speed_task::SpeedTaskBatch;
use rustle::task::TaskBatch;
use rustle::task::TaskOutput;
use rustle::db::DB;
use surrealdb::engine::any;
use tokio;
use tokio::sync::Mutex;


use rustle::config::Config;

use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();
    
    //===========RUN=THE=ALGORITHMS=========//

    //------ASYNC-BATCH------// (all running at the same time multiple times)

    let configs = config_setup().await;

    //let batch_task = TaskBatch{ configs: configs.clone(), iterations: 1, workers: 4};
    //let batch_output = batch_task.run().await.unwrap();

    let speed_task = SpeedTaskBatch{configs:configs.clone(),iterations:1,workers:4, speed_list: [1,1000,100,10].to_vec() };
    let out = speed_task.run().await;


    //============LOAD=FROM=FILE==============//
    //instead of running the algorithms from 0 you can load a previously saved database.

    //let batch_output = load_from_file().await;

    //===========COMPUTE=METRICS=============//

    //let batch_res = Metric::compute_batch(
    //    &batch_output, 
    //    EvoApeArg{
    //        //plot: Some(PlotArg::default()),
    //        plot: None,
    //        ..Default::default()
    //    }
    //);

    //__PRINT_IN_MD_TABLE___
    //let md_batch = Metric::print_batch(&batch_res);
    //println!("{}", md_batch);
    //______________________


    //_____PLOT_BOXPLOT_____
    //Metric::box_plot(&batch_res, "/home/mjpc13/Documents/rustle/test/results/box_plot_large.svg");
    //______________________

    //__PLOT_COMPUTER_STATS__
    //let mut task_vec: Vec<TaskOutput> = batch_output
    //    .into_iter()
    //    .map(|(k,v)| {
    //        v.into_iter().next().unwrap()
    //    })
    //    .collect();
    //task_vec.sort(); //sorts the algorithm to ensure that they have the same colors;
    //ContainerPlot::MemoryUsage.plot(&task_vec, "/home/mjpc13/Documents/rustle/test/results/memory_usage_large.svg");
    //ContainerPlot::LoadPercentage.plot(&task_vec, "/home/mjpc13/Documents/rustle/test/results/load_percentage_large.svg");

}

async fn config_setup() -> Vec<Config> {

    //Dataset related
    let dataset_path = "/home/mjpc13/Documents/rustle/test/dataset/";
    let gt_topic: &str = "/gt_poses";


    let db_connection = any::connect("file://test/db").await.unwrap();
    let test = db_connection.clone();


    let adv_ig = AdvancedConfig{
        db_connection: test,
        db_database: String::from("ig_lio"),
        docker_socket: Docker::connect_with_local_defaults().unwrap(),
    };

    //Put algorithms I want to run in a Vector
    let mut configs: Vec<Config> = Vec::new();

    //====ALGORITHMS=CONFIGS======//

    // iG-LIO //
    configs.push(
        Config::new(
            "mjpc13/rustle:ig-lio", //docker image name
            "iG-LIO", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mjpc13/Documents/rustle/test/config/ig-lio/params.yaml",  //Yaml config file for the algorithm 
            vec!["/lio_odom"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            //None
            Some(&adv_ig) //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    //----------------//

    let test = db_connection.clone();

    // LIO-SAM-6AXIS //
    let adv_lio_sam = AdvancedConfig{
        db_connection: test,
        db_database: String::from("lio_sam"),
        docker_socket: Docker::connect_with_local_defaults().unwrap(),
    };
    configs.push(
        Config::new(
            "mjpc13/rustle:lio-sam-6axis", 
            "LIO-SAM", 
            dataset_path, 
            "/home/mjpc13/Documents/rustle/test/config/lio-sam-6axis/params.yaml",   
            vec!["/lio_sam_6axis/mapping/odometry_incremental"], 
            gt_topic,  
            //None
            Some(&adv_lio_sam)
        ).await.unwrap()
    );

    //----------------//
    let test = db_connection.clone();

    let adv_ig = AdvancedConfig{
        db_connection: test,
        db_database: String::from("point_lio"),
        docker_socket: Docker::connect_with_local_defaults().unwrap(),
    };

    // POINT-LIO //
    configs.push(
        Config::new(
            "mjpc13/rustle:point-lio", 
            "Point-LIO", 
            dataset_path, 
            "/home/mjpc13/Documents/rustle/test/config/point-lio/params.yaml", 
            vec!["/aft_mapped_to_init"], 
            gt_topic,  
            None 
        ).await.unwrap()
    );

    // DLO //
    configs.push(Config::new(
        "mjpc13/rustle:dlo", //docker image name
        "DLO", //algorithm name
        dataset_path, // path to the directory of the rosbag
        "/home/mjpc13/Documents/rustle/test/config/dlo/params.yaml",  //Yaml config file for the algorithm 
        vec!["/dlo/odom_node/odom"], //Vector of topics to record
        gt_topic, //topic of the ground truth
        None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

   // DLIO //
    //configs.push(Config::new(
    //    "mjpc13/rustle:dlio", //docker image name
    //    "DLIO", //algorithm name
    //    dataset_path, // path to the directory of the rosbag
    //    "/home/mjpc13/Documents/rustle/test/config/dlio/params.yaml",  //Yaml config file for the algorithm 
    //    vec!["/dlio/odom_node/odom"], //Vector of topics to record
    //    gt_topic, //topic of the ground truth
    //    None //Advanced arguments (database and docker stuff) replace with None for default config
    //    ).await.unwrap()
    //);


    // SR-LIVO //
    //configs.push(Config::new(
    //        "mjpc13/rustle:sr-livo", //docker image name
    //        "SR-LIVO", //algorithm name
    //        dataset_path, // path to the directory of the rosbag
    //        "/home/mjpc13/Documents/rustle/test/config/sr-livo/params.yaml",  //Yaml config file for the algorithm 
    //        vec!["/Odometry_after_opt"], //Vector of topics to record
    //        gt_topic, //topic of the ground truth
    //        None //Advanced arguments (database and docker stuff) replace with None for default config
    //    ).await.unwrap()
    //);

    // A-LOAM //
    //configs.push(Config::new(
    //        "mjpc13/rustle:a-loam", //docker image name
    //        "Light-LOAM", //algorithm name
    //        dataset_path, // path to the directory of the rosbag
    //        "/home/mjpc13/Documents/rustle/test/config/a-loam/params.yaml",  //Yaml config file for the algorithm 
    //        vec!["/aft_mapped_to_init"], //Vector of topics to record
    //        gt_topic, //topic of the ground truth
    //        None //Advanced arguments (database and docker stuff) replace with None for default config
    //    ).await.unwrap()
    //);

    // Faster-LIO //
    //configs.push(Config::new(
    //        "mjpc13/rustle:faster-lio", //docker image name
    //        "Faster-LIO", //algorithm name
    //        dataset_path, // path to the directory of the rosbag
    //        "/home/mjpc13/Documents/rustle/test/config/faster-lio/params.yaml",  //Yaml config file for the algorithm 
    //        vec!["/Odometry"], //Vector of topics to record
    //        gt_topic, //topic of the ground truth
    //        None //Advanced arguments (database and docker stuff) replace with None for default config
    //    ).await.unwrap()
    //);

    return configs;

    //----------------//
}