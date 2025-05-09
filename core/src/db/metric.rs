use std::sync::Arc;

use log::{info, warn};
// db/metric.rs
use surrealdb::{Surreal, engine::local::Db};
use tokio::sync::Mutex;
use crate::models::metrics::pose_error::{Position, APE, RPE};
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

    pub async fn save_ape(&self, ape: &mut APE, iteration_id: &Thing) -> Result<(), DbError> {
        
        // Create metric record
        let created: Option<APE> = self.conn.lock().await
            .create("ape")
            .content(ape.clone())
            .await.unwrap();

        if let Some(created) = created {
            ape.id = created.id;
        }

        self.conn.lock().await
            .query("DEFINE INDEX ape_time_from_start ON ape FIELDS time_from_start")
            .await?;

        if let Some(ape_id) = &ape.id {
            self.conn.lock().await
                .query("RELATE $iteration->has_ape->$ape")
                .bind(("iteration", iteration_id.clone()))
                .bind(("ape", ape_id.clone()))
                .await.unwrap();
        }

        Ok(())
    }


    pub async fn save_rpe(&self, rpe: &mut RPE, iteration_id: &Thing) -> Result<(), DbError> {
        
        // Create metric record
        let created: Option<RPE> = self.conn.lock().await
            .create("rpe")
            .content(rpe.clone())
            .await.unwrap();

        if let Some(created) = created {
            rpe.id = created.id;
        }
        self.conn.lock().await
            .query("DEFINE INDEX rpe_time_from_start ON rpe FIELDS time_from_start")
            .await?;

        if let Some(rpe_id) = &rpe.id {
            self.conn.lock().await
                .query("RELATE $iteration->has_rpe->$rpe")
                .bind(("iteration", iteration_id.clone()))
                .bind(("rpe", rpe_id.clone()))
                .await.unwrap();
        }

        Ok(())
    }

    pub async fn save_position(&self, position: &mut Position, iteration_id: &Thing) -> Result<(), DbError> {
        
        // Create metric record
        let created: Option<Position> = self.conn.lock().await
            .create("position")
            .content(position.clone())
            .await.unwrap();

        if let Some(created) = created {
            position.id = created.id;
        }

        if let Some(position_id) = &position.id {
            self.conn.lock().await
                .query("RELATE $iteration->has_position->$position")
                .bind(("iteration", iteration_id.clone()))
                .bind(("position", position_id.clone()))
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
