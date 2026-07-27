from .base import BaseContainer
from utils import DockerWrapper

from pathlib import Path
from pydantic import BaseModel, Field
from typing import Dict

class PlayerConfig(BaseModel):
    bag_path: Path
    play_rate: float = Field(default=1.0, gt=0.0)
    delay: float = Field(default=1.0, gt=0.0)
    topic_remaps: Dict[str, str] = Field(default_factory=dict)


class PlayerContainer(BaseContainer):
    def __init__(self, config: PlayerConfig, docker: DockerWrapper, network_name: str, env: Dict[str, str]):
        super().__init__(docker, network_name, env)
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

        for original_topic, pipeline_topic in self.config.topic_remaps.items():
            command.extend(["--remap", f"{original_topic}:={pipeline_topic}"])
        
        self.container_id = self.docker.run_container(
            image="ros:jazzy-ros-base",
            command=command,
            volumes=volumes,
            environment=self.env,
            network_name=self.network_name
        )
        return self.container_id
