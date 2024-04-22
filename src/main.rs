use bollard::{API_DEFAULT_VERSION, Docker};
use plotters::prelude::*;

use rustle::db::DB;
use rustle::evo_wrapper;

use rustle::evo_wrapper::EvoApeArg;
use rustle::metrics::Metric;
use rustle::metrics::ContainerStats;
use rustle::task::AdvancedConfig;
use tokio;

use rustle::task::Config;
use rustle::task::Task;
// Use a connection function described above

use surrealdb::{
    engine::any
};


use surrealdb::engine::local::File;
use surrealdb::Surreal;

use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    let dataset_path = "/home/mario/Documents/rustle/test/dataset/";
    let params_path = "/home/mario/Documents/rustle/test/config/iG-LIO/";

    let topics: Vec<String> = vec![
       "/lio_odom".to_string(), 
    ];
    let gt_topic: String = String::from("/gt_poses");

    let adv = AdvancedConfig{
       // db_endpoint: String::from("file://test/db"),
        db_database: String::from("ig-lio"),
        ..Default::default()
    };

    let test = Config::new(
        "mjpc13/rustle:ig-lio".into(), 
        "ig-lio".into(), 
        dataset_path.into(), 
        params_path.into(), 
        topics,
        gt_topic, 
        Some(adv) // connect to a custom (and persistent) Surreal database. TODO: Don't pass a database but the endpoint!
    ).await;
    
    let task1: Task = Task::new(test.unwrap()).await;
    let result = task1.run().await.unwrap();

    let rpe = Metric::compute(
        result, 
        EvoApeArg::default(), 
        Some("/home/mario/Documents/rustle/test/results")
    );

    println!("{:?}", rpe);



//    //RUN EVO AND EXTRACT RESULTS
//    let groundtruth = "/home/mario/Documents/rustle/test/evo/groundtruth.txt";
//    let data = "/home/mario/Documents/rustle/test/evo/data.txt";
//    
//    //The test db;
//    let file = "file://test/db/";
//    let d = any::connect(file).await.unwrap();
//    d.use_ns("namespace").use_db("database").await.unwrap();
//
//    let db = DB {db: d};
//    let vec_stats: Vec<ContainerStats> = db.query_stats().await.unwrap(); 
//    let evo_args = EvoArgs::default();
//
//
//
//    let root = BitMapBackend::new("/home/mario/Documents/rustle/0.png", (1920, 1080)).into_drawing_area();
//    root.fill(&WHITE).unwrap();
//
//    let mut chart = ChartBuilder::on(&root)
//        .caption("Memory Usage (MiB)", ("sans-serif", 50).into_font())
//        .margin(5)
//        .x_label_area_size(50)
//        .y_label_area_size(50)
//        .build_cartesian_2d(0u32..562u32, 100f64..1000f64).unwrap();
//    
//
//    chart.configure_mesh().draw().unwrap();
//
//    chart
//        .draw_series(LineSeries::new(
//                vec_stats.into_iter().map(|s| 
//                    (s.uid.unwrap(), s.memory_stats.usage.unwrap() as f64 * 1e-6)
//                ),
//            //(-50..=50).map(|x| x as f64 / 50.0).map(|x| (x, x * x)),
//            &RED,
//        )).unwrap()
//        .label("LIO-SAM")
//        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
//
//    chart
//        .configure_series_labels()
//        .background_style(&WHITE.mix(0.8))
//        .border_style(&BLACK)
//        .draw().unwrap();
//
//    root.present().unwrap();
//
    




    //let m = evo_wrapper::evo_ape(groundtruth, data, EvoArgs::default());
    //let m = Metrics::compute(groundtruth, data, EvoArgs::default());
    
    //println!("{:?}", m.unwrap());

}
