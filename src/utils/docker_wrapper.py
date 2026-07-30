import logging
from pathlib import Path
from typing import Dict, Iterator, List, Any
import docker
from docker.errors import APIError, NotFound

logger = logging.getLogger(__name__)

class DockerWrapper:
    """Docker SDK helper wrapper, to make interaction with container easier."""

    def __init__(self):
        try:
            self.client = docker.from_env()
        except Exception as e:
            logging.critical(
                "Failed to connect to the Docker daemon. "
                "Is Docker running, and does your user have permissions to access the socket?"
            )
            raise e

    def create_shared_network(self, network_name: str):
        """Creates a Docker bridge network to the pipeline's container able to communicate together."""
        try:
            # Check if network already exists to avoid throwing unnecessary errors
            existing_networks = self.client.networks.list(names=[network_name])
            if existing_networks:
                logger.info(f"Docker network '{network_name}' already exists.")
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

    def remove_network(self, network_name: str) -> None:
        """Remove a custom network."""
        try:
            network = self.client.networks.get(network_name)
            logger.info(f"Removing Docker network: {network_name}")
            network.remove()
        except NotFound:
            logger.warning(f"Docker network '{network_name}' not found; skipping removal.")
        except APIError as e:
            logger.error(f"Failed to remove Docker network '{network_name}': {e}")
            raise e

    def run_container(
        self,
        image: str,
        command: List[str],
        volumes: Dict[Path, str],
        environment: Dict[str, str],
        network_name: str,
        **kwargs
    ) -> str:
        """Starts a container and returns its container ID.

        Parameters:
            image: docker image name to use
            command: command to execute in the container
            volumes: volume to mount as a:
                Dict[
                    Path: path to target directory on the host,
                    str: path to the mounted directory inside the container
                ]
            environment: environment variables to pass to the container as a:
                Dict[
                    str: environment variable key (e.g., 'ROS_DOMAIN_ID'),
                    str: environment variable value (e.g., '42')
                ]
            network_name: name of the network the container is part of
                *precond* the network needs to have been initialized
            **kwargs: additional keyword arguments passed directly to the Docker SDK 
                containers.run() method (e.g., nano_cpus, mem_limit, user)

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
                environment=environment,
                network=network_name,
                detach=True,
                stdin_open=True,
                tty=True,
                **kwargs
            )

            if container.id:
                logger.debug(f"Container was correctly spawned with id {container.id[:12]}")
                return container.id
            else:
                raise RuntimeError(f"Docker SDK returned an unexpected object type: {type(container)}")

        except APIError as e:
            logger.error(f"Failed to run container for image {image}: {e}")
            raise e

    def wait_for_container(self, container_id: str) -> int:
        """Blocks until the container exits and returns its exit code."""
        try:
            container = self.client.containers.get(container_id)
            logger.info(f"Waiting for container {container_id[:12]}.")
            result = container.wait()
            status = result.get("StatusCode", -1)
            if status != 0:
                logger.warning(f"Container '{container_id[:12]}' exited with status code {status}.")
            return status
        except NotFound as e:
            logger.error(f"Cannot wait for container {container_id[:12]}; it does not exist.")
            raise e
        except APIError as e:
            logger.error(f"Error waiting on container {container_id[:12]}: {e}")
            raise e

    def get_container_logs(self, container_id: str) -> str:
        """Retrieves current logs from the container as a string."""
        try:
            container = self.client.containers.get(container_id)
            return container.logs(stdout=True, stderr=True).decode("utf-8", errors="replace")
        except NotFound:
            logger.warning(f"Container {container_id[:12]} not found to fetch logs, returning empty logs.")
            return ""
        except APIError as e:
            logger.error(f"Error fetching container {container_id[:12]} logs: {e}")
            raise e

    def get_container_stats(self, container_id: str) -> Iterator[Dict[str, Any]]:
        """Get the stream of container stats (cpu, memory,...)"""
        try:
            container = self.client.containers.get(container_id)
            return container.stats(stream=True, decode=True)
        except NotFound:
            logger.warning(f"Container {container_id[:12]} not found to fetch logs, returning empty stats.")
            return []
        except APIError as e:
            logger.error(f"Error fetching container {container_id[:12]} stats: {e}")
            raise e

    def stop_and_remove_container(self, container_id: str) -> None:
        """Forces a container to stop and removes it."""
        container = None
        try:
            container = self.client.containers.get(container_id)
            logger.info(f"Stopping and removing container: {container_id[:12]}")
            
            container.stop(timeout=5)
            container.remove()
        except NotFound:
            logger.debug(f"Container {container_id[:12]} was already removed or never created.")
        except APIError as e:
            if container is None:
                raise RuntimeError(f"Unexpected containers.get() crash: {e}")
            logger.warning(f"Graceful stop failed for container {container_id[:12]}. Attempting force remove...")
            try:
                container.remove(force=True)
            except Exception as force_err:
                logger.error(f"Could not force remove container {container_id[:12]}: {force_err}")
                raise e

