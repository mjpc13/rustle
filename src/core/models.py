from pathlib import Path
from pydantic import BaseModel
from typing import List, Optional, Dict, Tuple


class DatasetConfig(BaseModel):
    name: str
    dataset_path: Path
    pointcloud_topic: str
    groundtruth_topic: str
    play_rate: float

class SlamConfig(BaseModel):
    name: str
    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str


class IterationConfig(BaseModel):
    dataset_config: DatasetConfig
    slam_config: SlamConfig

class IterationResult(BaseModel):
    monitoring: List[Dict[str, float]]
    ape: Dict[str, float]
    frame_rate: float


class PipelineConfig(BaseModel):
    dataset_configs: List[DatasetConfig]
    slam_configs: List[SlamConfig]

    iteration_repetion: int

PipelineResult = List[Tuple[str, List[IterationResult]]]
