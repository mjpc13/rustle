use crate::{db::stat::StatRepo, models::stat::ContainerStats, services::error::ProcessingError};

use surrealdb::sql::Thing;
#[derive(Clone)]
pub struct StatService {
    repo: StatRepo,
}

impl StatService {
    pub fn new(repo: StatRepo) -> Self {
        Self { repo }
    }

    pub async fn record_stats(
        &self,
        stats: ContainerStats,
        iteration_id: &Thing
    ) -> Result<ContainerStats, ProcessingError> {
        let mut stats = stats;
        self.repo.save(&mut stats, iteration_id)
            .await
            .map_err(|e| ProcessingError::Database(e))?;
        Ok(stats)
    }

    //pub async fn get_stats_for_iteration(
    //    &self,
    //    iteration_id: &Thing,
    //) -> Result<Vec<ContainerStats>, ProcessingError> {
    //    self.repo.get_by_iteration(iteration_id)
    //        .await
    //        .map_err(|e| ProcessingError::Database(e))
    //}




}