from pathlib import Path
import logging

from core.iteration import Iteration
from core.models import IterationConfig, DatasetConfig, SlamConfig
from utils import DockerInstance

logging.basicConfig(
        level=logging.INFO,
    )

def main():
    # Define the configuration for the iteration
    config = IterationConfig(
        dataset_config=DatasetConfig(
            name="BoxRotation",
            dataset_path=Path("./workspace/kiss_icp_demo/input_bag"),
            pointcloud_topic="/ouster/points",
            groundtruth_topic="/ground_truth",
            play_rate=1.0,
        ),

        steps=[SlamConfig(
            name="kiss-icp",
            algorithm_image="neorustle/kiss-icp:latest",
            algorithm_params=Path("./tests/kiss_icp_demo/params.yaml"),
            algorithm_package="kiss_icp",
            algorithm_node_name="kiss_icp_node",
            input_topic="/pointcloud_topic",
            output_topic="/kiss/odometry",
        )],

        monitor_idx=0,
    )

    network_name = "test_network"
    env = {
            "ROS_DOMAIN_ID": "42",
            "PYTHONUNBUFFERED": "1"
        }

    ape = None
    with DockerInstance(network_name, env) as docker:
        with Iteration(config, docker) as iteration:
            ape = iteration.run()

    # Print the computed APE
    print(f"Result: {ape}")

if __name__ == '__main__':
    main()

