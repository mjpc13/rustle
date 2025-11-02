use std::{collections::{BTreeMap, HashMap}, fs, path::Path};

use crate::{
    db::AlgorithmRunRepo, models::{algorithm_run::AlgorithmRun, metric::{Metric, StatisticalMetrics, StatisticalMetricsStamped}, metrics::{pose_error::{APE, RPE}, ContainerStats, CpuMetrics}, Algorithm, Iteration, TestDefinition, TestType}, services::error::ProcessingError, utils::{config::Config, plots::{algorithm_ape_line_chart, algorithm_cpu_load_chart, algorithm_memory_usage_chart, algorithm_plot, algorithm_rpe_line_chart, GraphType}}
};

use charming::Chart;
use chrono::{DateTime, Utc};
use futures_util::future::join_all;
use log::warn;
use surrealdb::sql::Thing;

use super::{error::{PlotError, RunError}, DbError, IterationService};

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
        test_type: TestType
    ) -> Result<AlgorithmRun, ProcessingError> {

        let algo = self.repo.get_algorithm(algorithm_id).await?;

        let mut run = AlgorithmRun::new(bag_speed, num_iterations, algo, test_type.clone(), test_execution_id.clone());
        self.repo.save(&mut run, test_execution_id, algorithm_id).await?;

        
        // Create The different iterations
        for i in 0..num_iterations {

            // Save each iteration
            let algo = self.repo.get_algorithm_by_run(&run).await?;

            match run.id.clone(){
                Some(thing) => {

                    self.iter_service.create(i, algo, &thing, test_execution_id, test_type.clone()).await?
                },
                None => warn!("ID of algorithm_run was empty")
            };
        }
        
        Ok(run)
    }

    pub async fn set_aggregate_metrics(&self, run: &AlgorithmRun) {

        let metric_list = self.repo.get_metrics(run).await.unwrap(); //get metrics associated with the algo run (metrics of the iterations)

        let aggregate_metrics = Metric::mean(metric_list);  //Compute the "mean for buckets of space" for CPU/Memory/APE/RPE;

        let _ = self.compute_buckets(run).await;

        for metric in aggregate_metrics{
            let _ = self.repo.update_aggregate_metric(run, metric).await;
        }
    }

    pub async fn get_iterations(&self, run: &AlgorithmRun) -> Result<Vec<Iteration>, DbError> {
        let iteration_list = self.repo.get_iterations(run).await;
        iteration_list
    }


    pub async fn plot(&self, run: &AlgorithmRun, path: &str, overwrite: bool, format:  &str, config: &Config) -> Result<HashMap<String, Chart>, PlotError>{

        let mut hash: HashMap<String, Chart> = HashMap::new();
        let iterations = self.repo.get_iterations(run).await.unwrap();

        let iter_path = self.get_parents_string(run).await.unwrap();
        let full_path = format!("{path}/{iter_path}");

        //Create the directories if they dont exist
        fs::create_dir_all(&full_path).unwrap();

        // Call the other plots
        let files = ["aggregated_cpu_load", "aggregated_memory_usage", "aggregated_ape", "aggregated_rpe"];

        for f in files{

            let filepath = format!("{}/{}.{}", full_path, f, format);

            if Path::new(&filepath).exists() && !overwrite {
                //Not sure if I should return here, or just emit a warning
                return Err(PlotError::FileExists(filepath));
            } else {
                let chart = match f {
                    "aggregated_cpu_load" => algorithm_plot(&run.cpu_load_list, config, GraphType::CPU),
                    "aggregated_memory_usage" => algorithm_plot(&run.mem_usage_list, config, GraphType::Memory),
                    "aggregated_ape" => algorithm_plot(&run.ape_list, config, GraphType::APE),
                    "aggregated_rpe" => algorithm_plot(&run.rpe_list, config, GraphType::RPE),
                    &_ => todo!("")
                };

                if let Ok(c) = chart {
                    hash.insert(filepath, c);
                }

            }

        }
        
        Ok(hash)
    }

    async fn plot_ape(&self, iterations: &Vec<Iteration>, test_def: &TestType, config: &Config) -> Result<Chart, PlotError>{

        let mut algo_ape: Vec<Vec<APE>> = Vec::new();

        for it in iterations{
            match self.iter_service.get_ape(it).await {
                Ok(ape_vec) => algo_ape.push(ape_vec),
                Err(_) => warn!("Iteration {} of container {} does not have APE values", it.iteration_num, it.container.image_name),
            }
        }

        //PLOTS
        algorithm_ape_line_chart(algo_ape, test_def, config)
    }

    async fn plot_rpe(&self, iterations: &Vec<Iteration>, test_def: &TestType, config: &Config) -> Result<Chart, PlotError>{

        let mut algo_rpe: Vec<Vec<RPE>> = Vec::new();

        for it in iterations{
            match self.iter_service.get_rpe(it).await {
                Ok(rpe_vec) => algo_rpe.push(rpe_vec),
                Err(_) => warn!("Iteration {} of container {} does not have APE values", it.iteration_num, it.container.image_name),
            }
        }

        algorithm_rpe_line_chart(algo_rpe, test_def,config)
    }

    async fn plot_cpu_load(&self, iterations: &Vec<StatisticalMetricsStamped>, config: &Config) -> Result<Chart, PlotError>{

        algorithm_cpu_load_chart(iterations, config)

    }

    async fn plot_memory_usage(&self, iterations: &Vec<StatisticalMetricsStamped>, config: &Config) -> Result<Chart, PlotError>{


        algorithm_memory_usage_chart(iterations, config)
    }

    async fn compute_buckets(&self, algo_run: &AlgorithmRun) -> Result<(), RunError> {

        let container_stats = self.get_all_container_stats(algo_run).await;
        let apes = self.get_all_ape(algo_run).await;
        let rpes = self.get_all_rpe(algo_run).await;

        let [cpu_load_list, mem_usage_list] = aggregate_comp(container_stats)?;
        let ape_list = aggregate_ape(apes)?;
        let rpe_list = aggregate_rpe(rpes)?;

        let _ = self.repo.update_cpu_load_list(algo_run, cpu_load_list).await;
        let _ = self.repo.update_mem_usage_list(algo_run, mem_usage_list).await;
        let _ = self.repo.update_ape_list(algo_run, ape_list).await;
        let _ = self.repo.update_rpe_list(algo_run, rpe_list).await;

        Ok(())
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
        self.repo.get_algorithm_by_run(algo_run).await
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

    pub async fn get_all_ape(&self, run: &AlgorithmRun) -> Vec<Vec<APE>>{

        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();

        let mut algo_ape: Vec<Vec<APE>> = Vec::new();

        for it in iterations{
            match self.iter_service.get_ape(&it).await {
                Ok(ape_vec) => algo_ape.push(ape_vec),
                Err(_) => (),
            }
        }

        algo_ape
    }

    pub async fn get_all_rpe(&self, run: &AlgorithmRun) -> Vec<Vec<RPE>>{

        //For each AlgorithmRun I need the container stats
        let iterations = self.repo.get_iterations(run).await.unwrap();

        let mut algo_rpe: Vec<Vec<RPE>> = Vec::new();

        for it in iterations{
            match self.iter_service.get_rpe(&it).await {
                Ok(rpe_vec) => algo_rpe.push(rpe_vec),
                Err(_) => warn!("Iteration {} of container {} does not have APE values", it.iteration_num, it.container.image_name),
            }
        }

        algo_rpe
    }

}

fn aggregate_ape(apes_list: Vec<Vec<APE>>) -> Result<Vec<StatisticalMetricsStamped>, RunError>{

    let mut time_buckets: BTreeMap<u32, Vec<f32>> = BTreeMap::new();

    for data in &apes_list {

        let time_sec: Vec<f32> = data.iter()
            .map(|ape| {
                ape.time_from_start
            })
            .collect();

        let ape_values: Vec<f32> = data.iter()
            .map(|ape| {
                ape.value
            })
            .collect();

        for (t, usage) in time_sec.into_iter().zip(ape_values) {
            let bucket_key = (t * 1000.0) as u32; // ms precision for alignment
            time_buckets
                .entry(bucket_key)
                .or_insert_with(Vec::new)
                .push(usage);
        }

    }    

    //For each i64 I need to compute the corresponding StatisticalMetricStamped
    let mut ape_list: Vec<StatisticalMetricsStamped> = time_buckets
        .iter()
        .map(|(key, apes)| {
            let time = (*key as f32) / 1000.0; // Convert back to seconds
            let stat = StatisticalMetrics::from_values(&apes, true).unwrap();
            
            StatisticalMetricsStamped { 
                stat, 
                timestamp: time
            }
        })
        .collect();
    
    ape_list.sort();

    Ok(ape_list)
}

fn aggregate_rpe(rpes_list: Vec<Vec<RPE>>) -> Result<Vec<StatisticalMetricsStamped>, RunError>{

    let mut time_buckets: BTreeMap<u32, Vec<f32>> = BTreeMap::new();

    for data in &rpes_list {

        let time_sec: Vec<f32> = data.iter()
            .map(|rpe| {
                rpe.time_from_start
            })
            .collect();

        let rpe_values: Vec<f32> = data.iter()
            .map(|rpe| {
                rpe.value
            })
            .collect();

        for (t, usage) in time_sec.into_iter().zip(rpe_values) {
            let bucket_key = (t * 1000.0) as u32; // ms precision for alignment
            time_buckets
                .entry(bucket_key)
                .or_insert_with(Vec::new)
                .push(usage);
        }
    }

    //For each i64 I need to compute the corresponding StatisticalMetricStamped
    let mut rpe_list: Vec<StatisticalMetricsStamped> = time_buckets
        .iter()
        .map(|(key, rpes)| {
            let time = (*key as f32) / 1000.0; // Convert back to seconds
            let stat = StatisticalMetrics::from_values(&rpes, true).unwrap();
            
            StatisticalMetricsStamped { 
                stat, 
                timestamp: time
            }
        })
        .collect();

    rpe_list.sort();

    Ok(rpe_list)

}

fn aggregate_comp(container_stats: Vec<Vec<ContainerStats>>) -> Result<[Vec<StatisticalMetricsStamped>;2], RunError>{

    let mut time_buckets_cpu: BTreeMap<u32, Vec<f32>> = BTreeMap::new();
    let mut time_buckets_mem: BTreeMap<u32, Vec<f32>> = BTreeMap::new();

    for iteration in &container_stats {
        let data_ts: Vec<DateTime<Utc>> = iteration.iter().map(|cs| cs.created_at).collect();
        let start_ts = data_ts[0];

        let time_sec: Vec<f32> = data_ts.iter()
            .map(|ts| (*ts - start_ts).num_seconds() as f32)
            .collect();

        let cpu_load: Result<Vec<f32>, RunError> = iteration.iter()
            .skip(2)
            .map(|cs| {
                
                let total_usage = cs.cpu_stats.cpu_usage.total_usage;
                let prev_usage = cs.precpu_stats.cpu_usage.total_usage;
                let system_cpu = cs.cpu_stats.system_cpu_usage.ok_or(RunError::Execution("CPU usage".to_owned()))?;
                let prev_system_cpu = cs.precpu_stats.system_cpu_usage.ok_or(RunError::Execution("Pre CPU usage".to_owned()))?;
                let online_cpus = cs.cpu_stats.online_cpus.ok_or(RunError::Execution("Online CPUs".to_owned()))? as f32;
                
                let used = (total_usage - prev_usage) as f32;
                let available = (system_cpu - prev_system_cpu) as f32;
                Ok(used / available * 100.0 * online_cpus)
                
            })
            .collect();

        let memory_usage: Vec<f32> = iteration.iter()
            .skip(2)
            .map(|cs| {

                let mu = cs.memory_stats.usage;

                let usage = match mu {
                    Some(u) => u,
                    None => 0
                };

                usage as f32 / 1_000_000.0 //Memory in MB

            })
            .collect();

        let cpu_load = cpu_load?;

        for (t, load) in time_sec.iter().zip(cpu_load) {
            let bucket_key = (t * 1000.0) as u32;
            time_buckets_cpu.entry(bucket_key).or_default().push(load);
        }

        for (t, mem) in time_sec.iter().zip(memory_usage) {
            let bucket_key = (t * 1000.0) as u32;
            time_buckets_mem.entry(bucket_key).or_default().push(mem);
        }
        
    }

    let mut cpu_list: Vec<StatisticalMetricsStamped> = time_buckets_cpu
        .iter()
        .map(|(key, loads)| {
            let time = (*key as f32) / 1000.0; // Convert back to seconds
            let stat = StatisticalMetrics::from_values(&loads, false).unwrap();
            
            StatisticalMetricsStamped { 
                stat, 
                timestamp: time
            }
        })
        .collect();

    let mut mem_list: Vec<StatisticalMetricsStamped> = time_buckets_mem
        .iter()
        .map(|(key, mems)| {
            let time = (*key as f32) / 1000.0; // Convert back to seconds
            let stat = StatisticalMetrics::from_values(&mems, false).unwrap();
            
            StatisticalMetricsStamped { 
                stat, 
                timestamp: time
            }
        })
        .collect();

    cpu_list.sort();
    mem_list.sort();

    Ok([cpu_list, mem_list])

}