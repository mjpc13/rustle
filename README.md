# neorustle

Neorustle is a modular containerized benchmark tool for ros2 SLAM algorithm.

## system architecture

```
Components
 |
 V
Docker
```

Each component correspond to a single docker container that execute one independant task such as dataset player or ros node spinner.
Different components share a Docker network that they use to comunicate together.

## setup

Every module used are defined in pyproject.toml, you can install all of them using:
```bash
pip install -e .
```

You also need docker running on your computer. For installation see on the official website.

## example

### kiss_icp_demo

This example is a minimalist demo for the following execution: player -> kiss-icp -> writer.

It feature:
- Dockerfile: to build the container that spin the kiss-icp node
- params.yaml: a parmeter file for the kiss-icp module
- rosbag_generator.py: a script that generate a simple rosbag that conain a groundtruth and a pointcloud topic.
  The generated data correspond to a sensor doing an 8m diameter circle in a 10m box 0.5m above the ground.
- main.py: the main script that run the three component and evo evaluation.

To run it you first have to build the docker image (only once):
```bash
docker build -t neorustle/kiss-icp:latest -f docker/Dockerfile.kiss_icp .
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
