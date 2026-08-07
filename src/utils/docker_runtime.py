import docker
from docker.errors import APIError, NotFound
from docker.models.containers import Container
from pathlib import Path
from typing import Dict, Iterator, List, Any, Optional

import logging
logger = logging.getLogger(__name__)

class DockerRuntime:
    """
    Manages the persistent Docker environment and with a shared network and environment.
    DockerRuntime should always be used inside of a `with` statement, or must be manually torn down.
    """

    def __init__(self, network_name: str, environment: Dict[str, str]):
        """
        Initialize a docker interface instance.

        Args:
            network_name: name of the network shared by all the containers created by this interface.
                This name should be unique enough to avoid collision.
            environment: Environment variables shared by the containers (Dict[VAR_NAME, value]).
        """
        self._network_name = network_name
        self._environment = environment
        try:
            self.client = docker.from_env()
        except Exception as e:
            logger.error(
                "Failed to connect to the Docker daemon. "
                "Is Docker running, and does your user have permissions to access the socket?"
            )
            raise e

        try:
            existing_networks = self.client.networks.list(names=[network_name])
            if existing_networks:
                logger.warning(f"Docker network '{network_name}' already exists.")
                return

            logger.info(f"Creating isolated Docker network: {network_name}")
            self.client.networks.create(
                name=network_name,
                driver="bridge",
                check_duplicate=True
            )
        except APIError as e:
            logger.error(f"Failed to create Docker network '{network_name}': {e}")
            raise e

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.teardown()

    def teardown(self):
        """
        Properly remove the docker resources used by the runtime.
        """
        # Remove the custom network.
        try:
            network = self.client.networks.get(self._network_name)
            logger.info(f"Removing Docker network: {self._network_name}")
            network.remove()
        except NotFound:
            logger.warning(f"Docker network '{self._network_name}' not found; skipping removal.")
        except APIError as e:
            logger.error(f"Failed to remove Docker network '{self._network_name}': {e}")
            raise e

    def run_container(
        self,
        image: str,
        command: List[str],
        volumes: Dict[Path, str],
        *,
        environment: Dict[str, str] = {},
        **kwargs
    ) -> str:
        """
        Starts a container and returns its container ID.

        Parameters:
            image: docker image name to use.
            command: command to execute in the container.
            volumes: volume to mount as a:
                Dict[
                    Path: path to target directory on the host,
                    str: path to the mounted directory inside the container
                ]
            environment: additional environment variables to pass to the container as a:
                Dict[
                    str: environment variable key,
                    str: environment variable value
                ]
            **kwargs: additional keyword arguments passed directly to the Docker SDK.

        Returns:
            str: The 64-character long unique hexadecimal container ID.
        """
        # Format volumes dict to what Docker SDK expects:
        # { '/absolute/host/path': {'bind': '/container/path', 'mode': 'rw'} }
        formatted_volumes = {}
        for host_path, container_path in volumes.items():
            abs_host_path = str(host_path.resolve())
            formatted_volumes[abs_host_path] = {
                "bind": container_path,
                "mode": "rw"
            }

        try:
            logger.info(f"Spawning container from image: {image}")
            container = self.client.containers.run(
                image=image,
                command=command,
                volumes=formatted_volumes,
                environment=environment | self._environment,
                network=self._network_name,
                detach=True,
                stdin_open=True,
                tty=True,
                **kwargs
            )

            assert container.id is not None
            logger.debug(f"Container was correctly spawned with id {container.id[:12]}")
            return container.id

        except APIError as e:
            logger.error(f"Failed to run container for image {image}: {e}")
            raise e

    def _get_container(self, container_id: str) -> Optional[Container]:
        try:
            return self.client.containers.get(container_id)
        except NotFound as e:
            return None
        except APIError as e:
            logger.error(f"Error fetching the container {container_id[:12]}: {e}")
            raise e

    def wait_for_container(self, container_id: str) -> int:
        """
        Blocks until the container exits and returns its exit code.
        """
        container = self._get_container(container_id)
        if container is None:
            logger.error(f"Cant wait for container: {container_id[:12]}; it does not exists.")
            raise NotFound(f"container {container_id[:12]}")

        logger.info(f"Waiting for container {container_id[:12]}.")
        result = container.wait()
        status = result.get("StatusCode", -1)
        if status != 0:
            logger.warning(f"Container '{container_id[:12]}' exited with status code {status}.")

        return status

    def get_container_logs(self, container_id: str) -> str:
        """
        Retrieves current logs from the container as a string.
        """
        container = self._get_container(container_id)
        if container is None:
            logger.warning(f"Container {container_id[:12]} not found to fetch logs, returning empty logs.")
            return ""

        return container.logs(stdout=True, stderr=True).decode("utf-8", errors="replace")

    def get_container_stats_stream(self, container_id: str) -> Iterator[Dict[str, Any]]:
        """
        Get the stream of container stats (cpu, memory,...).
        See the docker package documentation for more detail on the data structure.
        """
        container = self.client.containers.get(container_id)
        if container is None:
            logger.warning(f"Container {container_id[:12]} not found to fetch logs, returning empty stats.")
            return iter(())

        stats = container.stats(stream=True, decode=True)
        assert isinstance(stats, Iterator)

        return stats

    def stop_and_remove_container(self, container_id: str) -> None:
        """
        Forces a container to stop and removes it.
        """
        container = self._get_container(container_id)
        if container is None:
            logger.warning(f"Container {container_id[:12]} was already removed or never created.")
            return

        logger.info(f"Stopping and removing container: {container_id[:12]}")
        
        try:
            container.stop(timeout=5)
            container.remove()
        except APIError as e:
            logger.warning(f"Graceful stop failed for container {container_id[:12]}. Attempting force remove...")
            try:
                container.remove(force=True)
            except Exception as force_err:
                logger.error(f"Could not force remove container {container_id[:12]}: {force_err}")
                raise e

