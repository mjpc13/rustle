# neorustle

Neorustle is a modular containerized benchmark tool for ros2 SLAM algorithm.

## system architecture

### docker

`DockerInstance` object represent an execution environment. Any container created with the same instance can comunicate together.

### component

Any pipeline is divided into different component, each one corresponding to a container.
Components are then runned sequencially.

### iteration

An iteration correspond to a single execution of a pipeline.
It is defined by a dataset to play and an iteration step sequence.
This sequence should produce odometries, that are then evaluated.

### benchmark (wip)

A benchmark is a collection of steps that get composed (cartesian product) into iterations, and aggregate all the results.

## example

### kiss_icp_demo

This example is a minimalist demo for the following execution: player -> kiss-icp -> writer.

It feature:
- Dockerfile: to build the container that spin the kiss-icp node
- params.yaml: a parmeter file for the kiss-icp module
- rosbag_generator.py: a script that generate a simple rosbag that conain a groundtruth and a pointcloud topic.
  The generated data correspond to a sensor doing an 8m diameter circle in a 10m box 0.5m above the ground.
- main.py: the main script that run the pipeline.

To run it you first have to build the docker image (only once):
```bash
docker build -t neorustle/kiss-icp:latest -f tests/kiss_icp_demo/Dockerfile .
```

Then generate the bag (only once):
```bash
python tests/kiss_icp_demo/rosbag_generator.py
```

Then run the demo:
```bash
PYTHONPATH=src python tests/kiss_icp_demo/main.py
```

Every generated files can be found in workspace/kiss_icp_demo/.
