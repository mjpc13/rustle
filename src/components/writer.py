from .base import BaseContainer, BaseConfig
from utils import DockerInstance

import os
from pathlib import Path
from pydantic import Field
from typing import Dict, List

class WriterConfig(BaseConfig):
    output_dir: Path
    bag_name: str
    topics: List[str] = Field(min_length=1)

    def get_container(self, docker: DockerInstance) -> WriterContainer:
        return WriterContainer(self, docker)

class WriterContainer(BaseContainer):
    def __init__(self, config: WriterConfig, docker: DockerInstance):
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

        # make sure the writen file is owned by the host madhine
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
