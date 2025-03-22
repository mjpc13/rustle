use crate::{
    models::ros::{Odometry, RosMsg},
    db::OdometryRepo
};

use super::error::RosProcessingError;

pub struct OdometryService {
    repo: OdometryRepo,
}

impl OdometryService {
    pub fn new(repo: OdometryRepo) -> Self {
        Self { repo }
    }

    pub async fn process_ros_batch(
        &self,
        test_run_id: &str,
        messages: Vec<RosMsg>
    ) -> Result<(), RosProcessingError> {
        let odometries: Vec<Odometry> = messages
            .into_iter()
            .filter_map(|msg| match msg {
                RosMsg::Odometry(odom) => Some(odom),
                _ => None
            })
            .map(|mut odom| {
                odom.id = Some(format!("odometry:{}", ulid::Ulid::new()));
                odom.test_run_id = Some(test_run_id.to_string());
                odom
            })
            .collect();

        //self.repo.save_batch(&odometries)
        //    .await
        //    .map_err(|e| RosProcessingError::Storage(e.to_string()))?;
        
        Ok(())
    }

    //pub async fn get_trajectory(
    //    &self,
    //    test_run_id: &str
    //) -> Result<Vec<Odometry>, RosProcessingError> {
    //    self.repo.get_for_test_run(test_run_id)
    //        .await
    //        .map_err(|e| RosProcessingError::Query(e.to_string()))
    //}
}