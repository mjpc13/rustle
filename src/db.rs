use anyhow::{anyhow, Result};

use chrono::{DateTime, Utc};
use serde::{de, Deserialize, Serialize};
use std::{collections::HashMap, fmt::Debug, sync::Arc};

use tokio::sync::Mutex;

use surrealdb::{
    engine::any::{self, Any}, key, sql::{to_value, Array, Object, Thing, Value}, Response, Surreal
};

use bollard::container::{MemoryStats, CPUStats};
use crate::{metrics::ContainerStats, ros_msgs::{Header, Odometry, Pose, RosMsg, Twist}, task::TaskOutput};

use log::{debug, error, info, warn, trace};


#[derive(Debug, Serialize, Deserialize)]
struct Record{
    #[allow(dead_code)]
    id: Thing
}

#[derive(Debug, Clone)]
pub struct DB{
    pub db: Arc<Mutex<Surreal<Any>>>,
}
impl DB{
    pub async fn add_stat(&self, namespace: &str, task_id: &str, mut data: ContainerStats){

        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();
        
        data.created_at = Some(Utc::now());


        let create: Option<Record> = db_lock
            .create("stat")
            .content(data)
            .await
            .unwrap();

        let stats: Vec<Record> =  db_lock.select("stat").await.unwrap();
    }

    pub async fn query_stats(&self,  namespace: &str, task_id: &str) -> Result<Vec<ContainerStats>, surrealdb::Error> {    

        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();


        let mut groups = db_lock
            .query("SELECT * FROM type::table($table) ORDER BY uid;")
            .bind(("table", "stat"))
            .await.unwrap();

        let stats: Vec<ContainerStats> = groups.take(0)?;

        Ok(stats)
    }

    pub async fn add_odom(&self, namespace: &str, task_id: &str, mut data: Odometry, table: &str)
    {
        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();

        let create: Option<Record> = db_lock
            .create(table)
            .content(data)
            .await
            .unwrap();

    }

    pub async fn query_odom(&self, namespace: &str, task_id: &str, table: &str) -> Result<Vec<Odometry>, surrealdb::Error>
    {    
        let mut db_lock = self.db.lock().await;
        db_lock.use_ns(namespace).use_db(task_id).await.unwrap();

        let mut response = db_lock
            .query(format!("SELECT * FROM type::table(\"{}\") ORDER BY header", table))
            .await?;

        let odoms: Vec<_> = response.take(0)?;

        Ok(odoms)
    }

    pub async fn get_namespaces(&self) -> Result<Vec<String>, surrealdb::Error> {

        let mut db_lock = self.db.lock().await;

        let mut namespaces: Vec<String> = Vec::new();


        // Query the database for root information
        let mut response = db_lock.query("INFO FOR ROOT;").await?;
    
        // Extract the namespaces from the response
        let response_obj: Option<Value> = response.take(0)?; // Take the first result as an Object


        let response_json = match response_obj {
            Some(v) => v.into_json(),
            None => panic!("No value to be converted to JSON")
        };

        let namespaces_dic = response_json.get("namespaces");

        match namespaces_dic {
            Some(v) => {
                if let Some(obj) = v.as_object() {
                    // Iterate over the available namespaces.
                    for key in obj.keys() {

                        info!("Detected entry in database for: {:?}", key);

                        let cleaned_key = key.trim_matches('`').to_string(); //clean the backticks of the response
                        info!("Detected entry in database for: {:?}", cleaned_key);

                        namespaces.push(cleaned_key);
                    }

                }
            },
            None => {
                warn!("Could not query database! No SLAM algorithms entries found!")
            }
        }
    
        Ok(namespaces)
    }

}



//Code to load the experiments from files

async fn get_namespaces(db: Arc<Mutex<Surreal<Any>>>) -> Result<Vec<String>, surrealdb::Error> {

    let mut db_lock = db.lock().await;

    let mut namespaces: Vec<String> = Vec::new();


    // Query the database for root information
    let mut response = db_lock.query("INFO FOR ROOT;").await?;

    // Extract the namespaces from the response
    let response_obj: Value = response.take(0)?; // Take the first result as an Object
    
    println!("{:?}",response_obj);

    let response_json = response_obj.into_json();

    let namespaces_dic = response_json.get("namespaces");

    match namespaces_dic {
        Some(v) => {
            if let Some(obj) = v.as_object() {
                // Iterate over the available namespaces.
                for key in obj.keys() {

                    info!("Detected entry in database for: {:?}", key);

                    let cleaned_key = key.trim_matches('`').to_string(); //clean the backticks of the response
                    info!("Detected entry in database for: {:?}", cleaned_key);

                    namespaces.push(cleaned_key);
                }

            }
        },
        None => {
            warn!("Could not query database! No SLAM algorithms entries found!")
        }
    }

    Ok(namespaces)
}

async fn get_databases(db: Arc<Mutex<Surreal<Any>>>, namespace: String) -> Result<Vec<String>, surrealdb::Error>{

    let mut db_lock = db.lock().await;
    db_lock.use_ns(&namespace).await.unwrap();

    let mut databases: Vec<String> = Vec::new();

    let mut response: Value = db_lock.query("INFO FOR NS;").await?.take(0)?;
    let response_json = response.into_json();
    let databases_dic = response_json.get("databases");

    match databases_dic {
        Some(v) => {
            if let Some(obj) = v.as_object() {
                // Iterate over the available namespaces.
                for key in obj.keys() {


                    let cleaned_key = key.trim_matches('`').to_string(); //clean the backticks of the response
                    info!("Detected iteration in database {:?} for: {:?}", &namespace, cleaned_key);

                    databases.push(cleaned_key);
                }

            }
        },
        None => {
            warn!("No experiment found for SLAM algorithm {:?}!", &namespace);
        }
    }



    Ok(databases)

}

async fn get_tables(db: Arc<Mutex<Surreal<Any>>>, namespace: String, database_name: &str) -> Result<Vec<String>, surrealdb::Error> {

        let mut db_lock = db.lock().await;
        db_lock.use_ns(&namespace).use_db(database_name).await.unwrap();

        let mut tables: Vec<String> = Vec::new();


        let mut response: Value = db_lock.query("INFO FOR DB;").await?.take(0)?;
        let response_json = response.into_json();
        let tables_dic = response_json.get("tables");

        match tables_dic {
            Some(v) => {
                if let Some(obj) = v.as_object() {

                    for key in obj.keys() {

                        let cleaned_key = key.trim_matches('`').to_string(); //clean the backticks of the response    
                        tables.push(cleaned_key.to_string());
                    }

                }
            },
            None => {
                warn!("No experiment found for SLAM algorithm {:?}!", &namespace);
            }
        };

        if tables.len() < 3{
            warn!("Experiment {database_name} has {:} tables, the minimum should be 3 tables (stats, groundtruth and SLAM odometry). Data is missing!", tables.len());
        }

        Ok(tables)


}

pub async fn load_from_file() -> HashMap<String, Vec<TaskOutput>> {

    let db_connection = any::connect("file://test/db").await.unwrap();
    let connection = Arc::new(Mutex::new(db_connection));
    
    let db = DB { db: connection };

    let gt_topic = "groundtruth";

    let mut res_hash: HashMap<String, Vec<TaskOutput>> =  HashMap::new();

    let namespace_list = get_namespaces(db.db.clone()).await.unwrap(); //Get all namespaces

    //init new  in the dictionary
    for c in &namespace_list{
        res_hash.insert(c.to_string(), Vec::new());
    }

    //get all the db inside the namespaces
    for ns in &namespace_list{
        let database_list = get_databases(db.db.clone(), ns.clone()).await.unwrap();//get all db in namespace

        let mut task_output_vec = Vec::<TaskOutput>::new();

        for db_name in &database_list{

            let tables = get_tables(db.db.clone(), ns.clone(), db_name).await;

            let mut odoms_result = HashMap::<String, Vec<Odometry>>::new();
            let stats_result: Vec<ContainerStats> = db.query_stats(ns, db_name).await.unwrap();
    
            let  gt_result: Vec<Odometry> = db.query_odom(ns, db_name, gt_topic).await.unwrap();

            for t in tables.unwrap(){

                if !t.contains("stat") && !t.contains(gt_topic){

                    let odoms: Vec<Odometry> = db
                    .query_odom(ns, db_name, &t)
                    .await
                    .unwrap();
                
                    odoms_result.insert(t.to_string(), odoms);
                }
                
            }

            let to = TaskOutput{
                stats: stats_result,
                odoms: odoms_result,
                groundtruth: gt_result,
                name: db_name.to_string()
            };
            res_hash.get_mut(ns).unwrap().push(to);

        };

    };

    res_hash
}
