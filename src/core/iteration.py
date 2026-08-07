import logging
import secrets
import shutil
import tempfile
from pathlib import Path
from os import read
from typing import Optional, List

from .models import IterationConfig, IterationResult
from components import BaseComponent, BaseComponentConfig
from components import GenericNodeComponentConfig, PlayerComponentConfig, WriterComponentConfig
from utils import DockerRuntime, compute_ape, compute_frame_rate

logger = logging.getLogger(__name__)


class Iteration():
    """
    Class representing one iteration of a modular pipeline.
    Iteration should always be torndown or used in with statement.
    """

    iter_id: Optional[str] = None

    def __init__(self, config: IterationConfig, docker: DockerRuntime):
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

        topic_remap = "/pipeline/step_"

        player_config = self.config.dataset_config.to_component_config(topic_remap + "0")
        self.components: List[BaseComponent] = [player_config.to_component(self.docker)]
        for (i, step) in enumerate(self.config.steps):
            comp_conf = step.to_component_config(topic_remap + str(i), topic_remap + str(i + 1))
            self.components.append(comp_conf.to_component(self.docker))

        self.odom_topic = topic_remap + str(len(self.components) - 1)
        writer_config = WriterComponentConfig(
                output_dir=self.tmp_dir,
                bag_name=self.tmp_bag,
                topics=[self.odom_topic]
            )
        self.components.append(writer_config.to_component(self.docker))

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

            self.components[self.config.monitor_idx + 1].start_monitoring()
            self.components[0].wait()
            monitoring = self.components[self.config.monitor_idx + 1].stop_monitoring()

            for (i, component) in enumerate(self.components):
                logger.debug(f"Internal logs of component {i}")
                logger.debug(component.get_log())

        finally:
            for component in self.components:
                component.stop()

        frame_rate = compute_frame_rate(
                self.config.dataset_config.dataset_path,
                self.config.dataset_config.pointcloud_topic,
                self.tmp_dir / self.tmp_bag,
                self.odom_topic
            )

        ape = compute_ape(
                self.config.dataset_config.dataset_path,
                self.config.dataset_config.groundtruth_topic,
                self.tmp_dir / self.tmp_bag,
                self.odom_topic
            )

        return IterationResult(monitoring=monitoring, ape=ape, frame_rate=frame_rate)

