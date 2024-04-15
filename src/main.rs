use bollard::{API_DEFAULT_VERSION, Docker};


use rustle::db::DB;
use rustle::evo_wrapper;

use rustle::evo_wrapper::EvoArgs;
use rustle::metrics::Metrics;
use rustle::metrics::Stats;
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
    let params_path = "/home/mario/Documents/rustle/test/config/";

    let topics: Vec<String> = vec![
       "/lio_sam/mapping/odometry".to_string(), 
       "/lio_sam/mapping/odometry_incremental".to_string()
    ];

    //let test = Config::new(
    //    "mjpc13/rustle:lio-sam".into(), 
    //    dataset_path.into(), 
    //    params_path.into(), 
    //    topics, 
    //    None, //optionally we can pass a custom Docker socket
    //    None // connect to a custom (and persistent) Surreal database. TODO: Don't pass a database but the endpoint!
    //).await;
    
    //let task1: Task = Task::new(test.unwrap()).await;
    //let _result = task1.run().await.unwrap();

    //RUN EVO AND EXTRACT RESULTS
    let groundtruth = "/home/mario/Documents/rustle/test/evo/groundtruth.txt";
    let data = "/home/mario/Documents/rustle/test/evo/data.txt";
    
    //The test db;
    let file = "file://db/";
    let d = any::connect(file).await.unwrap();
    d.use_ns("namespace").use_db("database").await.unwrap();
    let db = DB {db: d};

    let vec_stats: Vec<Stats> = db.query_stats().await.unwrap(); 
    let evo_args = EvoArgs::default();

    //let m = evo_wrapper::evo_ape(groundtruth, data, EvoArgs::default());
    //let m = Metrics::compute(groundtruth, data, EvoArgs::default());
    
    //println!("{:?}", m.unwrap());

}
