use crate::{db::odometry::OdometryRepo, models::ros::{odometry::Odometry, ros_msg::RosMsg}, services::error::ProcessingError};

use log::warn;
use surrealdb::sql::Thing;

#[derive(Clone)]
pub struct RosService {
    odometry_repo: OdometryRepo,
}

impl RosService {
    pub fn new(odometry_repo: OdometryRepo) -> Self {
        Self { odometry_repo }
    }

    pub async fn process_message(
        &self,
        msg: RosMsg,
        iteration_id: &Thing
    ) -> Result<(), ProcessingError> {
        let odom = msg.as_odometry().unwrap();
        let mut db_odom = Odometry::new(odom.header);
        
        // Copy relevant fields
        db_odom.child_frame_id = odom.child_frame_id;
        db_odom.pose = odom.pose;
        db_odom.twist = odom.twist;

        self.odometry_repo.save(&mut db_odom, iteration_id).await?;
        Ok(())
    }
}