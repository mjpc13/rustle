from .base import BaseContainer
from utils import DockerWrapper

import os
from pathlib import Path
from pydantic import BaseModel, Field
from typing import Dict, List

class WriterConfig(BaseModel):
    output_dir: Path
    bag_name: str
    topics: List[str] = Field(min_length=1)

class WriterContainer(BaseContainer):
    def __init__(self, config: WriterConfig, docker: DockerWrapper, network_name: str, env: Dict[str, str]):
        super().__init__(docker, network_name, env)
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

        uid = os.getuid() if hasattr(os, "getuid") else 1000
        gid = os.getgid() if hasattr(os, "getgid") else 1000
        host_user_mapping = f"{uid}:{gid}"

        self.container_id = self.docker.run_container(
            image="ros:jazzy-ros-base",
            command=command,
            volumes=volumes,
            environment=self.env,
            network_name=self.network_name,
            user=host_user_mapping
        )
        return self.container_id
