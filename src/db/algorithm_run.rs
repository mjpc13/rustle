use std::sync::Arc;

use log::warn;
// db/algorithm_run.rs
use surrealdb::{engine::local::Db, sql::Thing, Response, Surreal};
use tokio::sync::Mutex;

use crate::{models::{Algorithm, AlgorithmRun}, services::DbError};

pub struct AlgorithmRunRepo {
    conn: Arc<Mutex<Surreal<Db>>>,
}

impl AlgorithmRunRepo {
    pub fn new(conn: Arc<Mutex<Surreal<Db>>>) -> Self {
        Self { conn }
    }

    pub async fn save(
        &self,
        run: &mut AlgorithmRun,
        test_execution_id: &Thing,
        algorithm_id: &Thing,
    ) -> Result<(), DbError> {

        //If slow add a Arc in here!
        
        let created: Option<AlgorithmRun> = self.conn.lock().await
            .create("algorithm_run")
            .content(run.clone())
            .await?;

        if let Some(created) = created {
            run.id = created.id;
        }

        let run_id = run.id.clone()
            .ok_or(DbError::NotFound("AlgorithmRun ID not found after creation".into()))?;

        // Create relationships
        self.conn.lock().await
            .query("RELATE $test_execution -> has_run -> $run")
            .bind(("test_execution", test_execution_id.clone()))
            .bind(("run", run_id.clone()))
            .await?;

        self.conn.lock().await
            .query("RELATE $run -> uses -> $algorithm")
            .bind(("run", run_id.clone()))
            .bind(("algorithm", algorithm_id.clone()))
            .await?;

        Ok(())
    }

    pub async fn get_algorithm(
        &self,
        run: &AlgorithmRun
    ) -> Result<Algorithm, DbError> {
        let run_id = run.id.clone()
            .ok_or(DbError::MissingField("AlgorithmRun ID"))?;

        let mut result = self.conn.lock().await
            .query("
                SELECT ->uses->algorithm.* AS algorithm 
                FROM $run_id
            ")
            .bind(("run_id", run_id.clone()))
            .await?;

        let algorithms: Option<Vec<Algorithm>> = result.take("algorithm")?;

        match algorithms{
            Some(a) => {
                a.into_iter().next()
                .ok_or_else(|| DbError::NotFound(
                    format!("Algorithm for run {} not found", run_id)
                ))
            },
            None => Err(DbError::NotFound(
                format!("Algorithm for run {} not found", run_id)
            ))
        }

    }

    pub async fn set_aggregate_metrics(&self, run: &AlgorithmRun){

        let run_id = run.id.clone()
        .ok_or(DbError::MissingField("AlgorithmRun ID")).unwrap();

        let result = self.conn.lock().await
            .query("
                SELECT
                math::mean(
                    array::union(
                        array::union(
                            (SELECT VALUE stats.mean FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'Cpu'),
                            (SELECT VALUE ape.mean FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        (SELECT VALUE rpe.mean FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                    )
                ) AS mean_of_means,

                math::max(
                    array::union(
                        array::union(
                            (SELECT VALUE stats.max FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'Cpu'),
                            (SELECT VALUE ape.max FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        (SELECT VALUE rpe.max FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                    )
                ) AS max_of_maxs,

                math::min(
                    array::union(
                        array::union(
                            (SELECT VALUE stats.min FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'Cpu'),
                            (SELECT VALUE ape.min FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        (SELECT VALUE rpe.min FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                    )
                ) AS min_of_mins,

                math::mean(
                    array::filter(
                        array::union(
                            (SELECT VALUE ape.rmse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError'),
                            (SELECT VALUE rpe.rmse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        $value != NONE
                    )
                ) AS mean_rmse,

                math::mean(
                    array::filter(
                        array::union(
                            (SELECT VALUE ape.sse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError'),
                            (SELECT VALUE rpe.sse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        $value != NONE
                    )
                ) AS mean_sse,

                math::stddev(
                    array::filter(
                        array::union(
                            (SELECT VALUE ape.rmse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError'),
                            (SELECT VALUE rpe.rmse FROM ->has_iteration->iteration->has_metric->metric WHERE metric_type = 'PoseError')
                        ),
                        $value != NONE
                    )
                ) AS std_rmse
                FROM $algorithm_run_id
            ")
            .bind(("run_id", run_id.clone()))
            .await.unwrap();

        warn!("My thing: {:?}", result);

    }


}



    //pub async fn get_for_test_execution(
    //    &self,
    //    test_execution_id: &str
    //) -> Result<Vec<AlgorithmRun>, DbError> {
    //    self.conn
    //        .query("SELECT * FROM algorithm_run WHERE test_execution_id = $test_execution_id")
    //        .bind(("test_execution_id", test_execution_id))
    //        .await?
    //        .take(0)
    //        .map_err(|e| DbError::Operation(e))
    //}

    //pub async fn get_for_algorithm(
    //    &self,
    //    algorithm_id: &str
    //) -> Result<Vec<AlgorithmRun>, DbError> {
    //    self.conn
    //        .query("SELECT * FROM algorithm_run WHERE algorithm_id = $algorithm_id")
    //        .bind(("algorithm_id", algorithm_id))
    //        .await?
    //        .take(0)
    //        .map_err(|e| DbError::Operation(e))
    //}


