from .base import BaseContainer, BaseConfig
from utils import DockerInstance

import logging
from pathlib import Path
from pydantic import Field
from typing import Dict, Optional

class GenericNodeConfig(BaseConfig):
    image: str
    package_name: str
    node_name: str
    params_file: Optional[Path]
    topic_remaps: Dict[str, str] = Field(default_factory=dict)

    def get_container(self, docker: DockerInstance) -> BaseContainer:
        return GenericNodeContainer(self, docker)


class GenericNodeContainer(BaseContainer):
    def __init__(self, config: GenericNodeConfig, docker: DockerInstance):
        super().__init__(docker)
        self.config = config

    def start(self) -> str:
        if self.config.params_file and not self.config.params_file.exists():
            raise FileNotFoundError(f"Params file path does not exist: {self.config.params_file}")

        volumes = {self.config.params_file: "/workspace/params.yaml"} if self.config.params_file else {}

        command = [
            "ros2", "run", self.config.package_name, self.config.node_name,
            "--ros-args",
            "-p", "use_sim_time:=True"
        ] 
        if self.config.params_file:
            command.extend(["--params-file", "/workspace/params.yaml"])

        for internal_topic, pipeline_topic in self.config.topic_remaps.items():
            command.extend(["-r", f"{internal_topic}:={pipeline_topic}"])

        self.container_id = self.docker.run_container(
            image=self.config.image,
            command=command,
            volumes=volumes,
        )
        return self.container_id
