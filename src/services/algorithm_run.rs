use std::collections::HashMap;

use crate::{
    db::AlgorithmRunRepo, models::{algorithm_run::{AlgorithmRun, RunAggregates}, metric::{Metric, MetricType}, metrics::{CpuMetrics, PoseErrorMetrics}}, services::error::ProcessingError
};

use log::warn;
use surrealdb::sql::Thing;

use super::IterationService;

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

    pub async fn set_aggregate_metrics(&self, run: &AlgorithmRun){

        let metric_list = self.repo.get_metrics(run).await.unwrap(); //get metrics associated with the algo run (metrics of the iterations)


        //Vec of pose metrics
        let pose_metrics: Vec<&PoseErrorMetrics> = metric_list.iter()
        .filter_map(|m| match &m.metric_type {
            MetricType::PoseError(p) => Some(p),
            _ => None
        })
        .collect();

        //Vec with only CPU metrics
        let cpu_metrics: Vec<&CpuMetrics> = metric_list.iter()
        .filter_map(|m| match &m.metric_type {
            MetricType::Cpu(p) => Some(p),
            _ => None
        })
        .collect();

        //Compute mean of vector of PoseMetrics
        let agg_pose = PoseErrorMetrics::mean(&pose_metrics);


        //Compute mean of vector of CPU metrics
        let agg_cpu = CpuMetrics::mean(&cpu_metrics);


        warn!("My agg pose for a run of: {:?}", agg_pose);









        //APPROACH DEUX
        //let groups = group_metrics(metric_list);
        // Access CPU metrics
        //if let Some(cpu_metrics) = groups.get::<CpuMetrics>() {
        //    // cpu_metrics is Vec<&CpuMetrics>
        //}

        //// Access pose error metrics
        //if let Some(pose_metrics) = groups.get::<PoseErrorMetrics>() {
        //    // pose_metrics is Vec<&PoseErrorMetrics>
        //}



        //warn!("My metrics: {:?}", metric_list);


    }


    //pub async fn add_iteration(
    //    &mut self,
    //    run: &mut AlgorithmRun,
    //    iteration: RunIteration
    //) -> Result<(), ProcessingError> {
    //    run.iterations.push(iteration);
    //    self.update_aggregates(run);
    //    //self.repo.save(run).await?;
    //    Ok(())
    //}

    //fn update_aggregates(&self, run: &mut AlgorithmRun) {
    //    let count = run.iterations.len() as f64;
    //    
    //    run.aggregates = RunAggregates {
    //        avg_ape: run.iterations.iter().map(|i| i.ape).sum::<f64>() / count,
    //        avg_rpe: run.iterations.iter().map(|i| i.rpe).sum::<f64>() / count,
    //        max_cpu: run.iterations.iter().map(|i| i.cpu_usage).fold(0.0, f32::max),
    //        avg_memory: run.iterations.iter().map(|i| i.memory_usage_mb).sum::<f32>() / count as f32,
    //        total_estimates: run.iterations.iter().map(|i| i.odometry_estimates.len()).sum(),
    //    };
    //    run.duration_secs = run.iterations.iter()
    //        .map(|i| i.odometry_estimates.last().map(|o| o.header.time))
    //        .filter_map(|t| t)
    //        .fold(0.0, |acc, t| {
    //            let secs = (t - run.created_at).num_seconds() as f64;
    //            secs.max(acc)
    //        });
    //}



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