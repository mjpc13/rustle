from abc import ABC, abstractmethod, abstractproperty
from typing import Dict, List, Optional
from utils import DockerWrapper
from pydantic import BaseModel
from time import sleep, time
import threading
import logging

logger = logging.getLogger(__name__)

class BaseConfig(ABC, BaseModel):
    """
    Abstract Base config for any component
    """

    @abstractmethod
    def get_container(self, docker: DockerWrapper, network_name: str, env: Dict[str, str]) -> BaseContainer:
        pass

class BaseContainer(ABC):
    """
    Abstract Base Class representing a modular pipeline step.
    """

    def __init__(self, docker: DockerWrapper, network_name: str, env: Dict[str, str]):
        self.container_id: Optional[str] = None

        self.docker: DockerWrapper = docker
        self.network_name: str = network_name
        self.env: Dict[str, str] = env

        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None
        self._stats: List[Dict[str, float]] = []

    @property
    def is_running(self) -> bool:
        return self.container_id is not None

    @abstractmethod
    def start(self) -> str:
        """
        Standardized interface to start the container.

        Returns:
            The 64-character container ID.
        """
        pass

    def stop(self) -> None:
        """
        Default, shared implementation for stopping containers.
        """
        if self.container_id is not None:
            self.docker.stop_and_remove_container(self.container_id)
            self.container_id = None

    def _monitor(self) -> None:
        """
        Monitor the container's stats. Should always run one a detached thread.

        Args:
            interval: The time interval between checks.
        """
        assert self.container_id is not None
        self._stop_event = threading.Event()

        stats = []
        time_origin = time()
        for raw_stat in self.docker.get_container_stats(self.container_id):
            if self._stop_event.is_set():
                break

            stat = {
                    "time": time() - time_origin,
                    "cpu": raw_stat['cpu_stats']['cpu_usage']['total_usage'],
                    "memory": raw_stat['memory_stats']['usage']
                }

            stats.append(stat)

        self._stats = stats

    def start_monitoring(self) -> None:
        """
        Start monitoring the container.
        """
        if self.container_id is None:
            logger.error("Can not start monitoring on a component that is not running")
            raise ValueError("container isn't running")
        if self._monitor_thread is not None:
            logger.warning("The monitoring thread is already running.")
            return

        logger.info("Starting monitoring thread.")
        self._monitor_thread = threading.Thread(target=self._monitor)
        self._monitor_thread.start()

    def stop_monitoring(self) -> List[Dict[str, float]]:
        """
        Stop the monitoring thread and return collected stats.
        """
        if self._monitor_thread is None:
            logger.error("The monitoring thread was not started.")
            raise ValueError("monitor_thread is None")
        if not self._monitor_thread.is_alive():
            logger.error("The monitoring thread has crashed for unknown reason.")
            raise RuntimeError("monitor_thread has crashed")

        assert self._stop_event is not None

        logger.info("Stoping monitoring thread.")
        self._stop_event.set()
        self._monitor_thread.join()
        self._monitor_thread = None

        return self._stats
