use crate::{db::MetricRepo, models::{Metric, MetricType}};
use super::error::ValidationError;

pub struct MetricService {
    repo: MetricRepo,
}

impl MetricService {
    pub fn new(repo: MetricRepo) -> Self {
        Self { repo }
    }

    //pub async fn record(&self, metric: Metric) -> Result<(), ValidationError> {
    //    self.validate(&metric)?;
    //    self.repo.save(&metric).await.map_err(|e| {
    //        ValidationError(format!("Failed to save metric: {}", e))
    //    })
    //}

    fn validate(&self, metric: &Metric) -> Result<(), ValidationError> {
        match &metric.metric_type {
            MetricType::Ape(stats) | MetricType::Rpe(stats) => {
                if stats.min > stats.max {
                    return Err(ValidationError(
                        "Minimum value cannot be greater than maximum".into()
                    ));
                }
                if stats.std_dev < 0.0 {
                    return Err(ValidationError(
                        "Standard deviation cannot be negative".into()
                    ));
                }
                Ok(())
            }
            MetricType::OutputFrequency(single) => {
                if single.value <= 0.0 {
                    Err(ValidationError(
                        "Output frequency must be positive".into()
                    ))
                } else {
                    Ok(())
                }
            }
        }
    }
}

