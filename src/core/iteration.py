import shutil
import tempfile
import logging
import secrets
from os import read
from pathlib import Path
from typing import Optional, List

from .models import IterationConfig, IterationResult
from components import BaseConfig, BaseContainer
from components import GenericNodeConfig, PlayerConfig, WriterConfig
from utils import DockerInstance, compute_ape, compute_drop_rate

logger = logging.getLogger(__name__)


class Iteration():
    """
    Class representing one iteration of a modular pipeline.
    Iteration should always be torndown or used in with statement.
    """

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

        self.component_configs: List[BaseConfig] = []

        self.player_idx = 0
        player_config = PlayerConfig(
                bag_path=config.dataset_config.dataset_path,
                topic_remaps={
                    config.dataset_config.groundtruth_topic: "/pipeline/groundtruth",
                    config.dataset_config.pointcloud_topic: "/pipeline/pointcloud",
                },
                play_rate=self.config.dataset_config.play_rate
            )
        self.component_configs.append(player_config)

        self.slam_idx = 1
        slam_config = GenericNodeConfig(
                image=config.slam_config.algorithm_image,
                params_file=config.slam_config.algorithm_params,
                package_name=config.slam_config.algorithm_package,
                node_name=config.slam_config.algorithm_node_name,
                topic_remaps={
                    config.slam_config.input_topic: "/pipeline/pointcloud",
                    config.slam_config.output_topic: "/pipeline/odometry",
                }
            )
        self.component_configs.append(slam_config)

        writer_config = WriterConfig(
                output_dir=self.tmp_dir,
                bag_name=self.tmp_bag,
                topics=["/pipeline/odometry"]
            )
        self.component_configs.append(writer_config)

        self.components: List[BaseContainer] = [c.get_container(self.docker) for c in self.component_configs]

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

            self.components[self.slam_idx].start_monitoring()
            self.components[self.player_idx].wait()
            monitoring = self.components[self.slam_idx].stop_monitoring()

            for (i, component) in enumerate(self.components):
                logger.debug(f"Internal logs of component {i}")
                logger.debug(component.get_log())

        finally:
            for component in self.components:
                component.stop()

        frame_rate = compute_drop_rate(
                self.config.dataset_config.dataset_path,
                self.config.dataset_config.pointcloud_topic,
                self.tmp_dir / self.tmp_bag,
                "/pipeline/odometry"
            )

        ape = compute_ape(
                self.config.dataset_config.dataset_path,
                self.config.dataset_config.groundtruth_topic,
                self.tmp_dir / self.tmp_bag,
                "/pipeline/odometry"
            )

        return IterationResult(monitoring=monitoring, ape=ape, frame_rate=frame_rate)

