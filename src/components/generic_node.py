from .base import BaseContainer
from utils import DockerWrapper

import logging
from pathlib import Path
from pydantic import BaseModel, Field
from typing import Dict, Optional

class GenericNodeConfig(BaseModel):
    image: str
    package_name: str
    node_name: str
    params_file: Optional[Path]
    topic_remaps: Dict[str, str] = Field(default_factory=dict)


class GenericNodeContainer(BaseContainer):
    def __init__(self, config: GenericNodeConfig, docker: DockerWrapper, network_name: str, env: Dict[str, str]):
        super().__init__(docker, network_name, env)
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
            environment=self.env,
            network_name=self.network_name
        )
        return self.container_id
