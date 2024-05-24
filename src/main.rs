
use std::result;

use rustle::evo_wrapper::EvoApeArg;
use rustle::evo_wrapper::PlotArg;
use rustle::metrics::ContainerPlot;
use rustle::metrics::Metric;
use rustle::metrics::ContainerStats;
use rustle::task::AdvancedConfig;
use rustle::task::TaskBatch;
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

    //====ALGORITHMS=CONFIGS======//

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

    //----------------//

    // LIO-SAM-6AXIS //
    configs.push(
        Config::new(
            "mjpc13/rustle:lio-sam-6axis", 
            "lio-sam-6axis", 
            dataset_path, 
            "/home/mario/Documents/rustle/test/config/lio-sam-6axis/params.yaml",   
            vec!["/lio_sam_6axis/mapping/odometry_incremental"], 
            gt_topic,  
            Some(&adv) 
        ).await.unwrap()
    );

    //----------------//

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

    //----------------//

    // POINT-LIO //
    configs.push(
        Config::new(
            "mjpc13/rustle:point-lio", 
            "point-lio", 
            dataset_path, 
            "/home/mario/Documents/rustle/test/config/point-lio/params.yaml", 
            vec!["/aft_mapped_to_init"], 
            gt_topic,  
            None 
        ).await.unwrap()
    );

    // DLO //
        configs.push(Config::new(
            "mjpc13/rustle:dlo", //docker image name
            "dlo", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/dlo/params.yaml",  //Yaml config file for the algorithm 
            vec!["/dlo/odom_node/odom"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );


    // SR-LIVO //
    configs.push(Config::new(
            "mjpc13/rustle:sr-livo", //docker image name
            "sr-livo", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/sr-livo/params.yaml",  //Yaml config file for the algorithm 
            vec!["/Odometry_after_opt"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    // A-LOAM //
    configs.push(Config::new(
            "mjpc13/rustle:a-loam", //docker image name
            "a-loam", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/a-loam/params.yaml",  //Yaml config file for the algorithm 
            vec!["/aft_mapped_to_init"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    // Faster-LIO //
    configs.push(Config::new(
            "mjpc13/rustle:faster-lio", //docker image name
            "faster-lio", //algorithm name
            dataset_path, // path to the directory of the rosbag
            "/home/mario/Documents/rustle/test/config/faster-lio/params.yaml",  //Yaml config file for the algorithm 
            vec!["/Odometry"], //Vector of topics to record
            gt_topic, //topic of the ground truth
            None //Advanced arguments (database and docker stuff) replace with None for default config
        ).await.unwrap()
    );

    //----------------//
    
    //========RUN=THE=ALGORITHMS=========//

    //------ASYNC-BATCH------// (all running at the same time multiple times)

    let batch_task = TaskBatch{ configs: configs.clone(), batch_size: 50, run_async: true };
    let batch_output = batch_task.run().await.unwrap();
    let batch_res = Metric::compute_batch(
        &batch_output, 
        EvoApeArg{
            plot: None,
            ..Default::default()
        }
    );
    println!("Finished batch:\n {:#?}", batch_res);
    //__PRINT_IN_MD_TABLE___
    let md_batch = Metric::print_batch(&batch_res);
    println!("{}", md_batch);
    //______________________
    //_____PLOT_BOXPLOT_____
    //Metric::box_plot(&batch_res, "/home/mario/Documents/rustle/test/results/box_plot.svg");
    //______________________



    //------SYNC-SINGLE------// (running 1 algo at a time only 1 time)
//    let mut single_to: Vec<TaskOutput> = Vec::<TaskOutput>::new();
//    for c in configs{
//        let task = Task::new(c.clone()).await;
//        let res = task.run().await.unwrap();
//        single_to.push(res);
//    }
//
//    //--COMPUTE-APE--
//    let evo_args = EvoApeArg{
//        //plot: Some(PlotArg::default()),
//        plot: None,
//        ..Default::default()
//    };
//
//    for to in single_to.clone(){
//        let ape = Metric::compute(
//            &to, 
//            &evo_args, 
//            None
//            //Some("/home/mario/Documents/rustle/test/results")
//        );
//        println!("APE: {:?}", ape);
//    }
//
//    //__PLOT_COMPUTER_STATS__
//    ContainerPlot::MemoryUsage.plot(&single_to, "/home/mario/Documents/rustle/test/results/memory_usage.svg");
//    ContainerPlot::LoadPercentage.plot(&single_to, "/home/mario/Documents/rustle/test/results/load_percentage.svg");
//    ContainerPlot::MemoryUsagePerSec.plot(&single_to, "/home/mario/Documents/rustle/test/results/memory_usage_per_sec.svg");

}