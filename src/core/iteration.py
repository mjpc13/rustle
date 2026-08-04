from os import read
import shutil
import tempfile
import logging
import secrets
from typing import Dict, List, Optional, Tuple, final
from pathlib import Path
from pydantic import BaseModel, Secret
from rosbags.rosbag2 import Reader

from components import BaseConfig, BaseContainer
from components import GenericNodeConfig, PlayerConfig, WriterConfig
from utils import DockerInstance, compute_ape

logger = logging.getLogger(__name__)

class IterationConfig(BaseModel):
    dataset_path: Path
    pointcloud_topic: str
    groundtruth_topic: str
    play_rate: float

    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str

    do_monitoring: bool


class IterationResult(BaseModel):
    monitoring: Optional[List[Dict[str, float]]]
    ape: Dict[str, float]
    frame_rate: float


class Iteration():
    """
    Class representing one iteration of a modular pipeline.
    Iteration should always be torndown or used in with statement.
    """

    network_name = "pipeline_network"
    iter_id: Optional[str] = None

    def __init__(self, config: IterationConfig, docker: DockerInstance):
        """
        Initialize the iteration with the given configuration.

        Args:
            config: Configuration for the iteration.
            docker: Docker instance for handling containers, this should outlive the iteration object.
        """
        self.config = config
        self.docker = docker

        self.iter_id = secrets.token_hex(6)
        logger.info(f"Initializing iteration: {self.iter_id}.")

        self.tmp_dir = Path(tempfile.mkdtemp(prefix=f"rustle_iteration_{self.iter_id}_"))
        self.tmp_bag = "output_bag"

        self.player_idx = 0
        player_config = PlayerConfig(
                bag_path=config.dataset_path,
                topic_remaps={
                    config.groundtruth_topic: "/pipeline/groundtruth",
                    config.pointcloud_topic: "/pipeline/pointcloud",
                },
                play_rate=self.config.play_rate
            )

        self.slam_idx = 1
        slam_config = GenericNodeConfig(
                image=config.algorithm_image,
                params_file=config.algorithm_params,
                package_name=config.algorithm_package,
                node_name=config.algorithm_node_name,
                topic_remaps={
                    config.input_topic: "/pipeline/pointcloud",
                    config.output_topic: "/pipeline/odometry",
                }
            )

        writer_config = WriterConfig(
                output_dir=self.tmp_dir,
                bag_name=self.tmp_bag,
                topics=["/pipeline/groundtruth", "/pipeline/pointcloud", "/pipeline/odometry"]
            )

        self.component_configs: List[BaseConfig] = [player_config, slam_config, writer_config]
        self.components = [c.get_container(self.docker) for c in self.component_configs]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.teardown()

    def teardown(self) -> None:
        """
        Teardown the iteration by removing the network and cleaning up temporary files.
        """
        if self.iter_id is None:
            logger.warning("This iteration was already torndown.")
            return

        logger.info(f"Trearing down iteration: {self.iter_id}")
        if self.tmp_dir.exists():
            shutil.rmtree(self.tmp_dir)

        self.iter_id = None

    def run(self) -> IterationResult:
        """
        Run iteration, monitoring, and evaluate results.

        Returns:
            The agregation of iteration measurement.
        """
        if self.iter_id is None:
            logger.error("You are trying to run a torndown iteration.")
            raise ValueError("can not run torndown iteration")

        try:
            for component in self.components[::-1]:
                component.start()

            if self.config.do_monitoring:
                self.components[self.slam_idx].start_monitoring()
            self.components[self.player_idx].wait()

            monitoring = None
            if self.config.do_monitoring:
                monitoring = self.components[self.slam_idx].stop_monitoring()

        finally:
            for component in self.components[::-1]:
                component.stop()

        gt_nb = -1.0
        with Reader(self.config.dataset_path) as reader:
            for connection in reader.connections:
                if connection.topic == self.config.groundtruth_topic:
                    gt_nb = connection.msgcount - 1

            assert gt_nb > 0

        frame_rate = -1.0
        with Reader(self.tmp_dir / self.tmp_bag) as reader:
            odom_nb = -1
            for connection in reader.connections:
                if connection.topic == "/pipeline/odometry":
                    odom_nb = connection.msgcount

            assert odom_nb != -1

            frame_rate = odom_nb / gt_nb

        ape = compute_ape(
            bag_path=self.tmp_dir / self.tmp_bag,
            gt_topic="/pipeline/groundtruth",
            odom_topic="/pipeline/odometry"
        )

        return IterationResult(monitoring=monitoring, ape=ape, frame_rate=frame_rate)

