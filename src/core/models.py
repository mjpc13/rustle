from abc import ABC, abstractmethod
from pathlib import Path
from pydantic import BaseModel
from typing import List, Optional, Dict, Tuple

from components import PlayerConfig, BaseConfig, GenericNodeConfig


class PipelineStep(ABC, BaseModel):
    @abstractmethod
    def to_component_config(self, input_remap: str, output_remap: str) -> BaseConfig:
        pass

class DatasetConfig(BaseModel):
    name: str
    dataset_path: Path
    pointcloud_topic: str
    groundtruth_topic: str
    play_rate: float

    def to_component_config(self, output_remap: str) -> PlayerConfig:
        return PlayerConfig(
                bag_path=self.dataset_path,
                topic_remaps={
                    self.pointcloud_topic: output_remap,
                },
                play_rate=self.play_rate
            )

class SlamConfig(PipelineStep):
    name: str
    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str

    def to_component_config(self, input_remap: str, output_remap: str) -> BaseConfig:
        return GenericNodeConfig(
                image=self.algorithm_image,
                params_file=self.algorithm_params,
                package_name=self.algorithm_package,
                node_name=self.algorithm_node_name,
                topic_remaps={
                    self.input_topic: input_remap,
                    self.output_topic: output_remap,
                }
            )


class IterationConfig(BaseModel):
    dataset_config: DatasetConfig
    steps: List[PipelineStep] 
    monitor_idx: int

class IterationResult(BaseModel):
    monitoring: List[Dict[str, float]]
    ape: Dict[str, float]
    frame_rate: float


class PipelineConfig(BaseModel):
    dataset_configs: List[DatasetConfig]
    slam_configs: List[SlamConfig]

    iteration_repetion: int

PipelineResult = List[Tuple[str, List[IterationResult]]]
