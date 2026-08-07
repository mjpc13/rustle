from utils import DockerRuntime

import threading
from abc import ABC, abstractmethod, abstractproperty
from pydantic import BaseModel
from time import sleep, time
from typing import Dict, List, Optional

import logging
logger = logging.getLogger(__name__)

class BaseComponentConfig(ABC, BaseModel):
    """
    Abstract Base config for any component.
    """

    @abstractmethod
    def to_component(self, docker: DockerRuntime) -> BaseComponent:
        """
        Instanciate and return the component configured by self and a given DockerRuntime.

        Args:
            docker: the docker runtime that should contain new component.
        """
        pass

class BaseComponent(ABC):
    """
    Abstract Base Class representing a modular pipeline component.

    A pipeline component manages a container lifecycle, giving access to stating stoping, and monitoring this one.
    """

    def __init__(self, docker: DockerRuntime):
        self.container_id: Optional[str] = None

        self.docker: DockerRuntime = docker

        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None
        self._stats: List[Dict[str, float]] = []

    @abstractmethod
    def start(self) -> str:
        """
        Standardized interface to start the container.
        When implementing this function be careful to set the attribute `container_id` before returning it.

        Returns:
            The 64-character container ID.
        """
        pass

    def stop(self) -> None:
        """
        Stop and remove the container.
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
        Get the log (stdout & stderr) of the container as a string.
        """
        if self.container_id is None:
            logger.warning("Container was never started or was removed, returning empty logs.")
            return ""

        return self.docker.get_container_logs(self.container_id)

    def _monitor(self) -> None:
        """
        Monitor the container's stats. Should always run one a detached thread.
        If you change the dict structure dont forget to update the `stop_monitoring` method docstring.
        """
        assert self.container_id is not None
        self._stop_event = threading.Event()

        stats = []
        time_origin = time()
        for raw_stat in self.docker.get_container_stats_stream(self.container_id):
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
        Start monitoring the container resource usage.
        The speed at which the data is sample depend on docker and is around once per second.
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

        Returns:
            List[{
                "time": time of the measure in ms since the start of the monitoring,
                "cpu": total cpu usage since the start of the container in ns of cpu,
                "memory": ram usage at the time of the measure in bytes,
            }]
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
