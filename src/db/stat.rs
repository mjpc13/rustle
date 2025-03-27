use std::sync::Arc;

use surrealdb::{engine::local::Db, Surreal, sql::Thing};
use tokio::sync::Mutex;
use crate::{models::stat::ContainerStats, services::error::DbError};

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

    //pub async fn get_by_iteration(&self, iteration_id: &Thing) -> Result<Vec<ContainerStats>, DbError> {
    //    self.conn
    //        .query("SELECT <-has_stat<-iteration.* FROM stat WHERE iteration_id = $iteration_id")
    //        .bind(("iteration_id", iteration_id))
    //        .await?
    //        .take(0)
    //}
}