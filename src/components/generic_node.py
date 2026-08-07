from .base import BaseComponent, BaseComponentConfig
from utils import DockerRuntime

from pathlib import Path
from pydantic import Field
from typing import Dict, Optional

class GenericNodeComponentConfig(BaseComponentConfig):
    """
    Config for generic ros2 node component.

    Attributes:
        image: docker image to run, this image should contain the node you're trying to run.
        package_name: name of the ros package you're trying to run.
        node_name: name of the node you're trying to run.
        params_file: parameter file path that is given to the node.
        topic_remaps: topic remaps as Dict[original topic name, new topic name]
    """
    image: str
    package_name: str
    node_name: str
    params_file: Optional[Path]
    topic_remaps: Dict[str, str] = Field(default_factory=dict)

    def to_component(self, docker: DockerRuntime) -> BaseComponent:
        return GenericNodeComponent(self, docker)


class GenericNodeComponent(BaseComponent):
    def __init__(self, config: GenericNodeComponentConfig, docker: DockerRuntime):
        super().__init__(docker)
        self.config = config

    def start(self) -> str:
        if self.config.params_file is not None and not self.config.params_file.exists():
            raise FileNotFoundError(f"Params file path does not exist: {self.config.params_file}")

        volumes = {self.config.params_file: "/workspace/params.yaml"} if self.config.params_file else {}

        command = [
            "ros2", "run", self.config.package_name, self.config.node_name,
            "--ros-args",
            "-p", "use_sim_time:=True"
        ] 
        if self.config.params_file is not None:
            command.extend(["--params-file", "/workspace/params.yaml"])

        for internal_topic, pipeline_topic in self.config.topic_remaps.items():
            command.extend(["-r", f"{internal_topic}:={pipeline_topic}"])

        self.container_id = self.docker.run_container(
            image=self.config.image,
            command=command,
            volumes=volumes,
        )
        return self.container_id
