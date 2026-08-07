from .models import IterationConfig, IterationResult, BenchmarkConfig, BenchmarkResult
from .iteration import Iteration
from utils import DockerRuntime


def run(config: BenchmarkConfig) -> BenchmarkResult:

    network_name = "test_network"
    env = {
            "ROS_DOMAIN_ID": "42",
            "PYTHONUNBUFFERED": "1"
        }

    pipline_results: BenchmarkResult = []

    with DockerRuntime(network_name, env) as docker:
        for dataset_config in config.dataset_configs:
            for slam_config in config.slam_configs:
                it_config = IterationConfig(dataset_config=dataset_config, steps=[slam_config], monitor_idx=0)
                identifier = f"{dataset_config.name}:{slam_config.name}"
                results = []
                with Iteration(it_config, docker) as iteration:
                    for _ in range(config.iteration_repetion):
                        results.append(iteration.run())

                pipline_results.append((identifier, results))

    return pipline_results
