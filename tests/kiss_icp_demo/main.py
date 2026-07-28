import logging
import shutil
from pathlib import Path

from components import PlayerConfig, PlayerContainer, GenericNodeConfig, GenericNodeContainer, WriterConfig, WriterContainer
from utils import DockerWrapper, compute_ape

### SETUP ###
workspace_dir = "./workspace/kiss_icp_demo"
input_bag_name = "input_bag"
output_bag_name = "output_bag"

if Path(f"{workspace_dir}/{output_bag_name}").resolve().exists():
    shutil.rmtree(f"{workspace_dir}/{output_bag_name}")

logging.basicConfig(
        level=logging.INFO,
    )


### CONFIGS ###
player_config = PlayerConfig(
        bag_path=Path(f"{workspace_dir}/{input_bag_name}").resolve(),
        topic_remaps={"/ouster/points": "/pipeline/raw_points"},
    )

slam_config = GenericNodeConfig(
        image="neorustle/kiss-icp:latest",
        package_name="kiss_icp",
        node_name="kiss_icp_node",
        params_file=Path("./tests/kiss_icp_demo/params.yaml"),
        topic_remaps={
            "/pointcloud_topic": "/pipeline/raw_points",
            "/kiss/odometry": "/pipeline/odometry"
        },
    )

writer_config = WriterConfig(
        output_dir=Path(f"{workspace_dir}").resolve(),
        bag_name=output_bag_name,
        topics=["/ground_truth", "/pipeline/odometry"],
    )


### DOCKER WRAPPER ###
wrapper = DockerWrapper()
wrapper.create_shared_network("network")

env = {
        "ROS_DOMAIN_ID": "42",
        "PYTHONUNBUFFERED": "1"
    }


### INSTANCES ###
player = PlayerContainer(
        config=player_config,
        docker=wrapper,
        network_name="network",
        env=env,
    )

slam = GenericNodeContainer(
        config=slam_config,
        docker=wrapper,
        network_name="network",
        env=env,
    )

writer = WriterContainer(
        config=writer_config,
        docker=wrapper,
        network_name="network",
        env=env,
    )


writer_id = writer.start()
slam_id = slam.start()
player_id = player.start()

wrapper.wait_for_container(player_id)

print("\n=== BEGIN LOG ===")
print("--- WRITER ---")
print(wrapper.get_container_logs(writer_id))
print("--- SLAM NODE ---")
print(wrapper.get_container_logs(slam_id))
print("=== END LOG ===\n")


### TEARDOWN ###
player.stop()
slam.stop()
writer.stop()

wrapper.remove_network("network")


### EVO APE ###
print("\n# Computing APE")
compute_ape(
        bag_path=Path(f"./workspace/kiss_icp_demo/output_bag").resolve(),
        gt_topic="/ground_truth",
        odom_topic="/pipeline/odometry",
        output_dir=Path(workspace_dir).resolve()
    )

print("done")
