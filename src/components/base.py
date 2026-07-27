from abc import ABC, abstractmethod
from typing import Dict, Optional
from utils import DockerWrapper

class BaseContainer(ABC):
    """Abstract Base Class representing a modular pipeline step."""

    def __init__(self, docker: DockerWrapper, network_name: str, env: Dict[str, str]):
        self.container_id: Optional[str] = None

        self.docker: DockerWrapper = docker
        self.network_name: str = network_name
        self.env: Dict[str, str] = env

    @property
    def is_running(self) -> bool:
        return self.container_id is not None

    @abstractmethod
    def start(self) -> str:
        """Standardized interface to start the container.
        
        Args:
            docker: The wrapper instance managing the daemon.
            network_name: The isolated network to join.
            env: Standardized environment parameters (e.g., ROS_DOMAIN_ID).
            
        Returns:
            The 64-character container ID.
        """
        pass

    def stop(self) -> None:
        """Default, shared implementation for stopping containers."""
        if self.container_id is not None:
            self.docker.stop_and_remove_container(self.container_id)
            self.container_id = None
