from abc import ABC, abstractmethod, abstractproperty
from typing import Dict, List, Optional
from utils import DockerInstance
from pydantic import BaseModel
from time import sleep, time
import threading
import logging

logger = logging.getLogger(__name__)

class BaseConfig(ABC, BaseModel):
    """
    Abstract Base config for any component.
    """

    @abstractmethod
    def get_container(self, docker: DockerInstance) -> BaseContainer:
        pass

class BaseContainer(ABC):
    """
    Abstract Base Class representing a modular pipeline step.
    """

    def __init__(self, docker: DockerInstance):
        self.container_id: Optional[str] = None

        self.docker: DockerInstance = docker

        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None
        self._stats: List[Dict[str, float]] = []

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
        if self.container_id is None:
            logger.warning("Container was never started or was removed, skipping stop.")
            return

        self.docker.stop_and_remove_container(self.container_id)
        self.container_id = None

    def wait(self) -> None:
        """
        Block on the container execution until it stop.
        """
        if self.container_id is None:
            logger.warning("Container was never started or was removed, skipping wait.")
            return

        if self.container_id is not None:
            self.docker.wait_for_container(self.container_id)

    def get_log(self) -> str:
        """
        Get the log (stdout & stderr) of the container.
        """
        if self.container_id is None:
            logger.warning("Container was never started or was removed, returning empty logs.")
            return ""

        return self.docker.get_container_logs(self.container_id)

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
