use std::{collections::HashMap, fs};

use crate::{
    db::AlgorithmRunRepo, models::{algorithm_run::AlgorithmRun, metric::{Metric, MetricType}, metrics::{pose_error::{APE, RPE}, ContainerStats, CpuMetrics, PoseErrorMetrics}, Algorithm, TestDefinition}, services::error::ProcessingError, utils::plots::{algorithm_ape_line_chart, algorithm_cpu_load_chart, algorithm_memory_usage_chart, algorithm_rpe_line_chart, memory_usage_line_chart}
};

use bollard::secret::ContainerState;
use directories::ProjectDirs;
use futures_util::future::join_all;
use log::warn;
use surrealdb::sql::Thing;

use super::{error::RunError, DbError, IterationService};

pub struct AlgorithmRunService {
    repo: AlgorithmRunRepo,
    iter_service: IterationService
}

impl AlgorithmRunService {
    pub fn new(repo: AlgorithmRunRepo, iter_service: IterationService) -> Self {
        Self { repo, iter_service }
    }

    pub async fn create_run(
        &self,
        bag_speed: f32,
        num_iterations: u8,
        test_execution_id: &Thing,
        algorithm_id: &Thing, 
        test_type: &str
    ) -> Result<AlgorithmRun, ProcessingError> {

        let mut run = AlgorithmRun::new(bag_speed, num_iterations);
        self.repo.save(&mut run, test_execution_id, algorithm_id).await?;

        
        // Create The different iterations
        for i in 0..num_iterations {

            // Save each iteration
            let algo = self.repo.get_algorithm(&run).await?;

            match run.id.clone(){
                Some(thing) => self.iter_service.create(i, algo, &thing, test_type.to_string()).await?,
                None => warn!("ID of algorithm_run was empty")
            };

        }

        Ok(run)
    }

    pub async fn set_aggregate_metrics(&self, run: &AlgorithmRun) {

        let metric_list = self.repo.get_metrics(run).await.unwrap(); //get metrics associated with the algo run (metrics of the iterations)

        let aggregate_metrics = Metric::mean(metric_list);

        for metric in aggregate_metrics{
            let _ = self.repo.update_aggregate_metric(run, metric).await;
        }


    }

    pub async fn plot_ape(&self, run: &AlgorithmRun){
        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();


        let algo_ape: Vec<Vec<APE>> = join_all(
            iterations.iter().map(|iter| async {
                self.iter_service.get_ape(iter).await.unwrap()
            })
        ).await;

        let algo_rpe: Vec<Vec<RPE>> = join_all(
            iterations.iter().map(|iter| async {
                self.iter_service.get_rpe(iter).await.unwrap()
            })
        ).await;

        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned())).unwrap();

            let iter_path = self.get_parents_string(run).await.unwrap();
            let full_path = format!("{data_dir_str}/{iter_path}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();

            //PLOTS
            let test_def: TestDefinition = self.repo.get_test_definition(&run).await.unwrap();
            algorithm_ape_line_chart(algo_ape, &test_def, &full_path);
            algorithm_rpe_line_chart(algo_rpe, &test_def, &full_path);


        };
    }

    pub async fn plot_cpu_load(&self, run: &AlgorithmRun){

        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();

        let algo_stats: Vec<Vec<ContainerStats>> = join_all(
            iterations.iter().map(|iter| async {
                self.iter_service.get_stats(iter).await.unwrap()
            })
        ).await;

        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned())).unwrap();

            let iter_path = self.get_parents_string(run).await.unwrap();
            let full_path = format!("{data_dir_str}/{iter_path}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();

            //PLOTS
            algorithm_cpu_load_chart(algo_stats, &full_path);

        };

    }

    pub async fn plot_memory_usage(&self, run: &AlgorithmRun){

        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();

        let algo_stats: Vec<Vec<ContainerStats>> = join_all(
            iterations.iter().map(|iter| async {
                self.iter_service.get_stats(iter).await.unwrap()
            })
        ).await;

        if let Some(proj_dirs) = ProjectDirs::from("org", "FRUC",  "RUSTLE") {
            let data_dir = proj_dirs.data_dir();

            let data_dir_str = data_dir.to_str().ok_or(RunError::Execution("Unable to fing application path".to_owned())).unwrap();


            //Create the folder for this iteration:
            // test_execution_id/algo_run_id/iteration_id/
            //something like self.repo.get_parents_id()
            let iter_path = self.get_parents_string(run).await.unwrap();
            let full_path = format!("{data_dir_str}/{iter_path}");

            //Create the directories if they dont exist
            fs::create_dir_all(&full_path).unwrap();
            algorithm_memory_usage_chart(algo_stats, &full_path);

        };

    }




    async fn get_parents_string(&self, algo_run: &AlgorithmRun) -> Result<String, RunError>{

            let te = self.repo.get_test_execution_thing(algo_run).await.unwrap();
    
            let algo_run_id = algo_run.id.clone()
                .ok_or_else(|| ProcessingError::NotFound("Iteration ID".into())).unwrap();
    
            let te_str = te.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
            let ar_str = algo_run_id.to_raw().replace(|c: char| !c.is_alphanumeric(), "_").to_lowercase();
    
    
            Ok(format!("{te_str}/{ar_str}"))
    }
    


    pub async fn get_algorithm(&self, algo_run: &AlgorithmRun) -> Result<Algorithm, DbError>{
        self.repo.get_algorithm(algo_run).await
    }

    pub async fn get_all_container_stats(&self, run: &AlgorithmRun) -> Vec<Vec<ContainerStats>>{
        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();
        let algo_stats: Vec<Vec<ContainerStats>> = join_all(
            iterations.iter().map(|iter| async {
                self.iter_service.get_stats(iter).await.unwrap()
            })
        ).await;
        algo_stats
    }

}



fn group_metrics(metrics: Vec<Metric>) -> MetricGroups {
    let mut groups = MetricGroups::default();
    for metric in metrics {
        groups.add(&metric);
    }
    groups
}


//A Struct to easily get the multiple metrics in a Vec<Metric>
#[derive(Default)]
struct MetricGroups {
    inner: HashMap<&'static str, Vec<Box<dyn std::any::Any>>>,
}

impl MetricGroups {
    pub fn add(&mut self, metric: &Metric) {
        let type_name = metric.metric_type.type_name();
        let entry = self.inner.entry(type_name).or_default();
        entry.push(Box::new(metric.metric_type.clone()));
    }

    pub fn get<T: 'static>(&self) -> Option<Vec<&T>> {
        self.inner.get(std::any::type_name::<T>())
            .map(|vec| vec.iter()
                .filter_map(|any| any.downcast_ref::<T>())
                .collect()
            )
    }
}