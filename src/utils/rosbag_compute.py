from pathlib import Path
from typing import Dict
from rosbags.rosbag2 import Reader
from evo.tools import file_interface
from evo.core import metrics, sync

import logging

logger = logging.getLogger(__name__)

def compute_drop_rate(
        pc_bag_path: Path,
        pc_topic: str,
        odom_bag_path: Path,
        odom_topic: str,
    ) -> float:
    """
    Compute the ratio of odometry over initial pointclouds message count.
    """
    pc_msg_count = -1
    with Reader(pc_bag_path) as reader:
        for connection in reader.connections:
            if connection.topic == pc_topic:
                pc_msg_count = connection.msgcount

        if pc_msg_count == -1:
            logger.error(f"The topic {pc_topic} was not found in the bag {pc_bag_path}")
            raise ValueError(f"topic {pc_topic} not found")

    odom_msg_count = -1
    with Reader(odom_bag_path) as reader:
        for connection in reader.connections:
            if connection.topic == odom_topic:
                odom_msg_count = connection.msgcount

        if odom_msg_count == -1:
            logger.error(f"The topic {odom_topic} was not found in the bag {odom_bag_path}")
            raise ValueError(f"topic {odom_topic} not found")

    pc_msg_count -= 1 # We remove one message, because the first message is always dropped
    # when playing the bag. There may be a fix, but I did not found it.

    if pc_msg_count <= 0:
        logger.warning("No message found in the pointcloud bag, returning a rate of 0.0.")
        return 0.0

    return odom_msg_count / pc_msg_count

def compute_ape(
        gt_bag_path: Path,
        gt_topic: str,
        odom_bag_path: Path,
        odom_topic: str,
    ) -> Dict[str, float]:
    """
    Compute the Average Pose Error (APE) between ground truth and odometry.
    """
    # Read ground truth and odometry trajectories from the ROS 2 bag
    logger.info("Reading trajectories.")
    with Reader(gt_bag_path) as reader:
        traj_ref = file_interface.read_bag_trajectory(reader, gt_topic)
    with Reader(odom_bag_path) as reader:
        traj_est = file_interface.read_bag_trajectory(reader, odom_topic)

    # Synchronize the trajectories based on timestamps
    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

    traj_est.align(traj_ref, correct_scale=False, correct_only_scale=False)

    # Compute APE using evo's metrics module
    logger.info("Computing ape.")
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref, traj_est))

    ape_stat = ape_metric.get_all_statistics()

    return ape_stat

