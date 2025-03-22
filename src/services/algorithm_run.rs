use crate::{
    models::algorithm_run::{AlgorithmRun, RunIteration, RunAggregates},
    db::AlgorithmRunRepo,
    services::error::ProcessingError
};

pub struct AlgorithmRunService {
    repo: AlgorithmRunRepo,
}

impl AlgorithmRunService {
    pub fn new(repo: AlgorithmRunRepo) -> Self {
        Self { repo }
    }

    pub async fn create_run(
        &self,
        test_execution_id: &str,
        algorithm_id: &str
    ) -> Result<AlgorithmRun, ProcessingError> {
        let run = AlgorithmRun::new(test_execution_id, algorithm_id);
        //self.repo.save(&run).await?;
        Ok(run)
    }

    pub async fn add_iteration(
        &mut self,
        run: &mut AlgorithmRun,
        iteration: RunIteration
    ) -> Result<(), ProcessingError> {
        run.iterations.push(iteration);
        self.update_aggregates(run);
        //self.repo.save(run).await?;
        Ok(())
    }

    fn update_aggregates(&self, run: &mut AlgorithmRun) {
        let count = run.iterations.len() as f64;
        
        run.aggregates = RunAggregates {
            avg_ape: run.iterations.iter().map(|i| i.ape).sum::<f64>() / count,
            avg_rpe: run.iterations.iter().map(|i| i.rpe).sum::<f64>() / count,
            max_cpu: run.iterations.iter().map(|i| i.cpu_usage).fold(0.0, f32::max),
            avg_memory: run.iterations.iter().map(|i| i.memory_usage_mb).sum::<f32>() / count as f32,
            total_estimates: run.iterations.iter().map(|i| i.odometry_estimates.len()).sum(),
        };

        run.duration_secs = run.iterations.iter()
            .map(|i| i.odometry_estimates.last().map(|o| o.header.time))
            .filter_map(|t| t)
            .fold(0.0, |acc, t| {
                let secs = (t - run.created_at).num_seconds() as f64;
                secs.max(acc)
            });
    }
}