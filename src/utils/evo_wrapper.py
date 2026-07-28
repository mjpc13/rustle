from pathlib import Path
from rosbags.rosbag2 import Reader
from evo.tools import file_interface
from evo.core import metrics, sync

import logging
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)

def compute_ape(
        bag_path: Path,
        gt_topic: str,
        odom_topic: str,
        output_dir: Path,
    ):
    """Computes Absolute Pose Error using evo."""
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_path = output_dir / "ape_plot.png"

    # Read ground truth and odometry trajectories from the ROS 2 bag
    with Reader(bag_path) as reader:
        traj_ref = file_interface.read_bag_trajectory(reader, gt_topic)
        traj_est = file_interface.read_bag_trajectory(reader, odom_topic)

    # Synchronize the trajectories based on timestamps
    max_diff = 0.01
    traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est, max_diff)

    traj_est.align(traj_ref, correct_scale=False, correct_only_scale=False)

    # Compute APE using evo's metrics module
    logger.info("Computing ape...")
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_ref, traj_est))

    ape_stat = ape_metric.get_all_statistics()
    logger.info("Compute success.")

    print(f"APE stats: {ape_stat}")

