use surrealdb::{Surreal, engine::remote::ws::Client};
use crate::models::Metric;

pub struct MetricRepo {
    conn: Surreal<Client>,
}

impl MetricRepo {
    pub fn new(conn: Surreal<Client>) -> Self {
        Self { conn }
    }

    //pub async fn save(&self, metric: &Metric) -> Result<(), surrealdb::Error> {
    //    self.conn
    //        .create(("metric", &metric.id))
    //        .content(metric)
    //        .await
    //}

    //pub async fn get_for_analysis(
    //    &self,
    //    test_run_id: &str,
    //    algorithm_id: &str,
    //    metric_type: &str
    //) -> Result<Vec<Metric>, surrealdb::Error> {
    //    self.conn
    //        .query("SELECT * FROM metric 
    //                WHERE test_run_id = $test_run_id 
    //                AND algorithm_id = $algorithm_id 
    //                AND metric_type.type = $metric_type")
    //        .bind(("test_run_id", test_run_id))
    //        .bind(("algorithm_id", algorithm_id))
    //        .bind(("metric_type", metric_type))
    //        .await?
    //        .take(0)
    //}
}