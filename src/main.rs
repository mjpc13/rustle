use tokio;

use rustle::Config;
use rustle::Task;
// Use a connection function described above


use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    //Hardcode paths
    let dataset_path = "/home/mjpc13/Documents/rustle/test/dataset/";
    let params_path = "/home/mjpc13/Documents/rustle/test/config/";
    
    let test = Config::new("mjpc13/rustle:lio-sam".to_string(), dataset_path.to_string(), params_path.to_string()).await;
    let task1: Task = Task::new(test.unwrap()).await;

    task1.run().await;

}
