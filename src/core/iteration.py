import time
import shutil
import tempfile
import logging
import secrets
from typing import Dict, List, Optional, Tuple, final
from pathlib import Path
from pydantic import BaseModel, Secret

from components import (
        GenericNodeConfig, GenericNodeContainer,
        PlayerConfig, PlayerContainer,
        WriterConfig, WriterContainer
    )
from utils import DockerWrapper, compute_ape

logger = logging.getLogger(__name__)

class IterationConfig(BaseModel):
    dataset_path: Path
    pointcloud_topic: str
    groundtruth_topic: str

    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str

    do_monitoring: bool


class Iteration():
    """
    Class representing one iteration of a modular pipeline.
    """

    network_name = "pipeline_network"
    iter_id: Optional[str] = None

    def __init__(self, config: IterationConfig):
        """
        Initialize the iteration with the given configuration.

        Args:
            config: Configuration for the iteration.
        """
        self.config = config

        self.iter_id = secrets.token_hex(6)
        logger.info(f"Initializing iteration: {self.iter_id}.")

        self.tmp_dir = Path(tempfile.mkdtemp(prefix=f"rustle_iteration_{self.iter_id}_"))
        self.tmp_bag = "output_bag"

        self.player_config = PlayerConfig(
                bag_path=config.dataset_path,
                topic_remaps={
                    config.groundtruth_topic: "/pipeline/groundtruth",
                    config.pointcloud_topic: "/pipeline/pointcloud",
                }
            )

        self.slam_config = GenericNodeConfig(
                image=config.algorithm_image,
                params_file=config.algorithm_params,
                package_name=config.algorithm_package,
                node_name=config.algorithm_node_name,
                topic_remaps={
                    config.input_topic: "/pipeline/pointcloud",
                    config.output_topic: "/pipeline/odometry",
                }
            )

        self.writer_config = WriterConfig(
                output_dir=self.tmp_dir,
                bag_name=self.tmp_bag,
                topics=["/pipeline/groundtruth", "/pipeline/pointcloud", "/pipeline/odometry"]
            )


        self.docker_wrapper = DockerWrapper()
        self.docker_wrapper.create_shared_network(self.network_name)

        env = {
                "ROS_DOMAIN_ID": "42",
                "PYTHONUNBUFFERED": "1"
            }

        self.player = PlayerContainer(
                config=self.player_config,
                docker=self.docker_wrapper,
                env=env,
                network_name=self.network_name,
            )

        self.slam = GenericNodeContainer(
                config=self.slam_config,
                docker=self.docker_wrapper,
                env=env,
                network_name=self.network_name,
            )

        self.writer = WriterContainer(
                config=self.writer_config,
                docker=self.docker_wrapper,
                env=env,
                network_name=self.network_name,
            )

    def run(self, verbose:bool = False) -> Tuple[Optional[List[Dict[str, float]]], Dict[str, float]]: # TODO: make a proper reult object
        """
        Run the iteration and compute APE.

        Args:
            verbose: if true print the log of all the containers.

        Returns:
            (Slam container monitoring, Computed APE).
        """
        if self.iter_id is None:
            logger.error("You are trying to run a torndown iteration.")
            raise ValueError("Can not run torndown iteration")

        try:
            writer_id = self.writer.start()
            slam_id = self.slam.start()
            player_id = self.player.start()

            if self.config.do_monitoring:
                self.slam.start_monitoring()
            self.docker_wrapper.wait_for_container(player_id)
            monitoring = None
            if self.config.do_monitoring:
                monitoring = self.slam.stop_monitoring()

            if verbose:
                print("== LOG ==")
                print("-- writer --")
                print(self.docker_wrapper.get_container_logs(writer_id))
                print("-- slam --")
                print(self.docker_wrapper.get_container_logs(slam_id))
                print("-- player --")
                print(self.docker_wrapper.get_container_logs(player_id))
                print("== END LOG ==")

        finally:
            self.player.stop()
            self.slam.stop()
            self.writer.stop()

        ape = compute_ape(
            bag_path=self.tmp_dir / self.tmp_bag,
            gt_topic="/pipeline/groundtruth",
            odom_topic="/pipeline/odometry"
        )

        return (monitoring, ape)

    def teardown(self) -> None:
        """
        Teardown the iteration by removing the network and cleaning up temporary files.
        """
        if self.iter_id is None:
            logger.warning("This iteration was already torndown.")
            return

        logger.info(f"Trearing down iteration: {self.iter_id}")
        self.docker_wrapper.remove_network(self.network_name)
        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir)
