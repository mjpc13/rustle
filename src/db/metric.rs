use std::sync::Arc;

use log::{info, warn};
// db/metric.rs
use surrealdb::{Surreal, engine::local::Db};
use tokio::sync::Mutex;
use crate::services::error::DbError;
use crate::models::metrics::Metric;
use surrealdb::sql::Thing;

#[derive(Clone)]
pub struct MetricRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl MetricRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(&self, metric: &mut Metric, iteration_id: &Thing) -> Result<(), DbError> {
        // Create metric record
        let created: Option<Metric> = self.conn.lock().await
            .create("metric")
            .content(metric.clone())
            .await.unwrap();

        if let Some(created) = created {
            metric.id = created.id;
        }

        if let Some(metric_id) = &metric.id {
            self.conn.lock().await
                .query("RELATE $iteration->has_metric->$metric")
                .bind(("iteration", iteration_id.clone()))
                .bind(("metric", metric_id.clone()))
                .await.unwrap();
        }

        Ok(())
    }

    //pub async fn get_for_iteration(
    //    &self,
    //    iteration_id: &Thing
    //) -> Result<Vec<Metric>, DbError> {
    //    self.conn
    //        .query("
    //            SELECT <-has_metric<-iteration.* 
    //            FROM metric 
    //            WHERE <-has_metric<-iteration INSIDE $iter_id
    //        ")
    //        .bind(("iter_id", iteration_id.clone()))
    //        .await?
    //        .take(0)
    //}
}
