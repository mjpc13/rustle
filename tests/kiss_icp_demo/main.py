from pathlib import Path
import logging

from core.iteration import Iteration, IterationConfig

logging.basicConfig(
        level=logging.INFO,
    )

def main():
    # Define the configuration for the iteration
    config = IterationConfig(
        dataset_path=Path("./workspace/kiss_icp_demo/input_bag"),
        pointcloud_topic="/ouster/points",
        groundtruth_topic="/ground_truth",

        algorithm_image="neorustle/kiss-icp:latest",
        algorithm_params=Path("./tests/kiss_icp_demo/params.yaml"),
        algorithm_package="kiss_icp",
        algorithm_node_name="kiss_icp_node",
        input_topic="/pointcloud_topic",
        output_topic="/kiss/odometry"
    )

    iteration = Iteration(config)

    try:
        ape = iteration.run()
    finally:
        iteration.teardown()

    # Print the computed APE
    print(f"Computed APE: {ape}")

if __name__ == '__main__':
    main()
