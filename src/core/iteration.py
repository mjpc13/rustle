from typing import Optional
from pathlib import Path
from pydantic import BaseModel

class IterationConfig(BaseModel):
    dataset_path: Path
    groundtruth_topic: str

    algorithm_image: str
    algorithm_params: Optional[Path]
    algorithm_package: str
    algorithm_node_name: str
    input_topic: str
    output_topic: str


class Iteration():
    def __init__(self, config: IterationConfig):
        self.config = config

