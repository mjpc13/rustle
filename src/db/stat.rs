use std::sync::Arc;

use log::warn;
use surrealdb::{engine::local::Db, Surreal, sql::Thing};
use tokio::sync::Mutex;
use crate::{models::{metrics::stat::ContainerStats, Iteration}, services::{error::DbError, iteration}};

#[derive(Clone)]
pub struct StatRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl StatRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, stat: &mut ContainerStats, iteration_id: &Thing) -> Result<(), DbError> {

        
        let created: Option<ContainerStats> = self.conn.lock().await
            .create("stat")
            .content(stat.clone())
            .await?;

        if let Some(created) = created {
            stat.id = created.id;
        }

        // Create relationship with iteration
        if let Some(stat_id) = stat.clone().id {
            self.conn.lock().await
                .query("RELATE $iteration->has_stat->$stat")
                .bind(("iteration", iteration_id.clone()))
                .bind(("stat", stat_id))
                .await?;
        }

        Ok(())
    }

    pub async fn get_by_iteration(&self, iteration: &Iteration) -> Result<Vec<ContainerStats>, DbError> {
        
        let iteration_id = iteration.id.clone()
        .ok_or(DbError::MissingField("Iteration ID"))?;

        warn!("POIS {:?}", iteration_id);


        let mut result = self.conn.lock().await
            .query("SELECT 
                        (SELECT * FROM $iteration_id->has_stat->stat ORDER BY created_at ASC) AS stats
                    FROM $iteration_id"
                )
            .bind(("iteration_id", iteration_id.clone()))
            .await.unwrap();


        let stats_opt: Option<Vec<ContainerStats>> = result.take("stats").unwrap();

        warn!("My result for stats: {:?}", stats_opt);

        stats_opt.ok_or(DbError::NotFound("No stat was found for iteration.".to_owned()))

        }
}