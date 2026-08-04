from pathlib import Path
from pydantic import BaseModel
from typing import List, Optional

from components import BaseConfig

class DatasetConfig(BaseModel):
    dataset_path: Path
    pointcloud_topic: str
    groundtruth_topic: str
    play_rate: float

class SlamConfig(BaseModel):
    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str

class PipelineConfig(BaseModel):
    dataset_configs: List[DatasetConfig]
    slam_configs: List[SlamConfig]

    iteration_repetion: int

class IterationConfig(BaseModel):
    dataset_config: DatasetConfig
    slam_config: SlamConfig

    do_monitoring: bool

