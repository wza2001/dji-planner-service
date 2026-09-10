from enum import Enum
from typing import List, Optional
from pydantic import BaseModel
import math

class LensType(str, Enum):
    single = "single"
    oblique_5lens = "oblique_5lens"
    multi_spectrum = "multi_spectrum"

class PayloadSpec(BaseModel):
    name: str
    payload_enum: int
    has_gimbal: bool
    lens_type: LensType
    sensor_width_mm: float
    sensor_height_mm: float
    focal_length_mm: float
    fixed_pitch_deg: float = -90.0

    @property
    def fov_h(self) -> float:
        return 2 * math.degrees(math.atan(self.sensor_width_mm / (2 * self.focal_length_mm)))

    @property
    def fov_v(self) -> float:
        return 2 * math.degrees(math.atan(self.sensor_height_mm / (2 * self.focal_length_mm)))

class DroneSpec(BaseModel):
    name: str
    drone_enum: int
    drone_sub_enum: int
    supported_payload_ids: List[str]
