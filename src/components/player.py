from .base import BaseComponent, BaseComponentConfig
from utils import DockerRuntime

from pathlib import Path
from pydantic import Field
from typing import Dict

class PlayerComponentConfig(BaseComponentConfig):
    """
    Config for a ros2 bag player component.

    Attributes:
        bag_path: path to the rosbag.
        play_rate: play rate of the bag (default: 0.0)
        delay: delay in sec before the bag start playing after the component is started (default: 1.0)
        topic_remaps: topic remaps as Dict[original topic name, new topic name]
    """
    bag_path: Path
    play_rate: float = Field(default=1.0, gt=0.0)
    delay: float = Field(default=1.0, gt=0.0)
    topic_remaps: Dict[str, str] = Field(default_factory=dict)

    def to_component(self, docker: DockerRuntime) -> PlayerComponent:
        return PlayerComponent(self, docker)


class PlayerComponent(BaseComponent):
    def __init__(self, config: PlayerComponentConfig, docker: DockerRuntime):
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
