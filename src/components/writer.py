from .base import BaseComponent, BaseComponentConfig
from utils import DockerRuntime

import os
from pathlib import Path
from pydantic import Field
from typing import Dict, List

class WriterComponentConfig(BaseComponentConfig):
    """
    Config for a rosbag recorder component.

    Attributes:
        output_dir: path to the directory in which the bag should be written.
        bag_name: name of the new bag.
        topics: names of the topics to listen to.
    """
    output_dir: Path
    bag_name: str
    topics: List[str] = Field(min_length=1)

    def to_component(self, docker: DockerRuntime) -> WriterComponent:
        return WriterComponent(self, docker)

class WriterComponent(BaseComponent):
    def __init__(self, config: WriterComponentConfig, docker: DockerRuntime):
        super().__init__(docker)
        self.config = config

    def start(self) -> str:

        self.config.output_dir.mkdir(parents=True, exist_ok=True)
        volumes = {self.config.output_dir: "/workspace/output"}

        command = [
            "ros2", "bag", "record",
            "-o", "/workspace/output/" + self.config.bag_name,
            "--use-sim-time",
            "--topics"
        ] + self.config.topics

        # make sure the written file is owned by the host madhine
        uid = os.getuid() if hasattr(os, "getuid") else 1000
        gid = os.getgid() if hasattr(os, "getgid") else 1000
        host_user_mapping = f"{uid}:{gid}"

        self.container_id = self.docker.run_container(
            image="ros:jazzy-ros-base",
            command=command,
            volumes=volumes,
            user=host_user_mapping
        )
        return self.container_id
