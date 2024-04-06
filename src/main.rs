use tokio;

use rustle::task::Config;
use rustle::task::Task;
// Use a connection function described above

use env_logger::init;

#[tokio::main]
async fn main() {
    
    init();

    //Hardcode paths
    let dataset_path = "/home/mjpc13/Documents/rustle/test/dataset/";
    let params_path = "/home/mjpc13/Documents/rustle/test/config/";
    
    let test = Config::new("mjpc13/rustle:lio-sam".into(), dataset_path.into(), params_path.into()).await;
    let task1: Task = Task::new(test.unwrap()).await;

    task1.run().await;

}
