use bollard::Docker;
use chrono::{DateTime, Utc};
use serde::{Serialize, Deserialize};
use crate::models::{Algorithm, ros::Odometry};
use surrealdb::sql::Thing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Iteration {
    pub id: Option<Thing>,
    pub iteration_num: u8,
    pub container: DockerContainer,
    pub created_at: DateTime<Utc>,
    //pub docker_socket: Arc<Docker>
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DockerContainer{
    pub image_name: String,
    pub container_name: String,
}

impl Iteration {
    pub fn new(iteration_num: u8, container: DockerContainer) -> Self {

        let docker = Docker::connect_with_local_defaults()
        .map_err(|e| format!("Failed to connect to Docker: {}", e)).unwrap();

        // Connect to SurrealDB This probably will have to be inside a Arc<Mutex>
        //let docker = Arc::new(docker);

        Self {
            id: None,
            iteration_num,
            container,
            created_at: Utc::now(),
            //docker_socket: docker
        }
    }
}