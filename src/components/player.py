from .base import BaseConfig, BaseContainer
from utils import DockerInstance

from pathlib import Path
from pydantic import Field
from typing import Dict

class PlayerConfig(BaseConfig):
    bag_path: Path
    play_rate: float = Field(default=1.0, gt=0.0)
    delay: float = Field(default=1.0, gt=0.0)
    topic_remaps: Dict[str, str] = Field(default_factory=dict)

    def get_container(self, docker: DockerInstance) -> PlayerContainer:
        return PlayerContainer(self, docker)


class PlayerContainer(BaseContainer):
    def __init__(self, config: PlayerConfig, docker: DockerInstance):
        super().__init__(docker)
        self.config = config

    def start(self) -> str:
        if not self.config.bag_path.exists():
            raise FileNotFoundError(f"Dataset bag path does not exist: {self.config.bag_path}")

        volumes = {self.config.bag_path: "/workspace/dataset"}
        
        command = [
            "ros2", "bag", "play",
            "/workspace/dataset",
            "--clock",
            "--rate", str(self.config.play_rate),
            "--delay", str(self.config.delay),
        ]

        if self.config.topic_remaps.items():
            command.append("--remap")
        for original_topic, pipeline_topic in self.config.topic_remaps.items():
            command.extend([f"{original_topic}:={pipeline_topic}"])
        
        self.container_id = self.docker.run_container(
            image="ros:jazzy-ros-base",
            command=command,
            volumes=volumes,
        )
        return self.container_id
