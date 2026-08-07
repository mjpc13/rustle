from pathlib import Path

from core.iteration import Iteration
from core.models import IterationConfig, DatasetConfig, SlamConfig
from utils import DockerRuntime

import logging
logging.basicConfig(
        level=logging.INFO,
    )

def main():
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
    with DockerRuntime(network_name, env) as docker:
        with Iteration(config, docker) as iteration:
            result = iteration.run()

    print(f"Result: {result}")

if __name__ == '__main__':
    main()

