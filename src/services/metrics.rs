use crate::{
    db::metric::MetricRepo,
    models::{metric::{Metric, MetricType, StatisticalMetrics}, metrics::{CpuMetrics, PoseErrorMetrics}, Iteration},
    services::error::ProcessingError
};

use chrono::Utc;
use surrealdb::sql::Thing;


#[derive(Clone)]
pub struct MetricService {
    repo: MetricRepo,
}

impl MetricService {
    pub fn new(repo: MetricRepo) -> Self {
        Self { repo }
    }

    // Generic save method
    pub async fn save(&self, metric: &mut Metric, iteration_id: &Thing) -> Result<(), ProcessingError> {

        self.repo.save(metric, iteration_id)
            .await
            .map_err(|e| ProcessingError::Database(e))
    }

    // Pose error-specific creation
    pub async fn create_pose_error_metric(
        &self,
        iteration_id: Thing,
        ape_stats: StatisticalMetrics,
        rpe_stats: StatisticalMetrics
    ) -> Result<Metric, ProcessingError> {
        
        let mut metric = Metric{
            id: None,
            metric_type: MetricType::PoseError(PoseErrorMetrics {
                ape: ape_stats,
                rpe: rpe_stats,
                created_at: Utc::now(),
            })
        };
        
        self.repo.save(&mut metric, &iteration_id).await?;
        Ok(metric)
    }


    pub async fn create_cpu_metric(
        &self,
        iteration_id: Thing,
        cpu_metric: CpuMetrics,
    ) -> Result<Metric, ProcessingError> {
        
        let mut metric = Metric{
            id: None,
            metric_type: MetricType::Cpu(cpu_metric)
        };
        
        self.repo.save(&mut metric, &iteration_id).await?;
        Ok(metric)
    }

    pub async fn create_freq_metric(
        &self,
        iteration_id: Thing,
        freq: StatisticalMetrics,
    ) -> Result<Metric, ProcessingError> {
        
        let mut metric = Metric{
            id: None,
            metric_type: MetricType::Frequency(freq)
        };
        
        self.repo.save(&mut metric, &iteration_id).await?;
        Ok(metric)
    }




    fn compute_stats(values: &[f64], is_pose_error: bool) -> StatisticalMetrics {
        let mut sorted = values.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let mean = sorted.iter().sum::<f64>() / sorted.len() as f64;
        let variance = sorted.iter()
            .map(|v| (v - mean).powi(2))
            .sum::<f64>() / sorted.len() as f64;
        let std_dev = variance.sqrt();

        StatisticalMetrics {
            mean,
            median: sorted[sorted.len() / 2],
            min: *sorted.first().unwrap_or(&0.0),
            max: *sorted.last().unwrap_or(&0.0),
            std: std_dev,
            rmse: is_pose_error.then(|| (values.iter().map(|v| v.powi(2)).sum::<f64>() / values.len() as f64).sqrt()),
            sse: is_pose_error.then(|| values.iter().map(|v| v.powi(2)).sum()),
        }
    }
}