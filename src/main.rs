use bollard::{API_DEFAULT_VERSION, Docker};
use rustle::evo_wrapper;
// use rustle::evo_wrapper::EvoArgs;
use tokio;

use rustle::task::Config;
use rustle::task::Task;
// Use a connection function described above

use surrealdb::{
    engine::any
};


use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    let dataset_path = "/home/mjpc13/Documents/rustle/test/dataset/";
    let params_path = "/home/mjpc13/Documents/rustle/test/config/";

    let topics: Vec<String> = vec![
       "/lio_sam/mapping/odometry".to_string(), 
       "/lio_sam/mapping/odometry_incremental".to_string()
    ];


    //Supply Docker socket AND Database
    
    let _docker = Docker::connect_with_local_defaults().unwrap();

    let endpoint = std::env::var("SURREALDB_ENDPOINT").unwrap_or_else(|_| "memory".to_owned());
    let db = any::connect(endpoint).await.unwrap();
    db.use_ns("namespace").use_db("database").await.unwrap();

    let test = Config::new("mjpc13/rustle:lio-sam".into(), dataset_path.into(), params_path.into(), topics, None, None).await;
    let task1: Task = Task::new(test.unwrap()).await;

    task1.run().await; //Needs to returs a struct with Vec<Stats> and Vec<Odometry>...



    // 
    // let groundtruth = "/home/mario/Documents/rustle/test/evo/groundtruth.txt";
    // let data = "/home/mario/Documents/rustle/test/evo/data.txt";
    //
    // let evo_args = EvoArgs::default();
    // println!("{}", evo_args);
    //
    // let m = evo_wrapper::evo_ape(groundtruth, data, EvoArgs::default());
    //
    // println!("{:?}", m.unwrap());

}
