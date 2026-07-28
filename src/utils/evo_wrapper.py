from pathlib import Path
from typing import Dict
from rosbags.rosbag2 import Reader
from evo.tools import file_interface
from evo.core import metrics, sync

import logging

logger = logging.getLogger(__name__)

def compute_ape(
        bag_path: Path,
        gt_topic: str,
        odom_topic: str,
    ) -> Dict[str, float]:
    """
    Compute the Average Pose Error (APE) between ground truth and odometry.

    Args:
        bag_path: Path to the ROS bag file.
        gt_topic: Topic name for ground truth data.
        odom_topic: Topic name for odometry data.

    Returns:
        Dictionary containing computed APE values.
    """
    # Read ground truth and odometry trajectories from the ROS 2 bag
    logger.info("Reading trajectories.")
    with Reader(bag_path) as reader:
        traj_ref = file_interface.read_bag_trajectory(reader, gt_topic)
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
    logger.info("Compute success.")

    return ape_stat

