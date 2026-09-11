from pydantic import BaseModel
from typing import List, Dict, Any

class GenericWaypoint(BaseModel):
    index: int
    lon: float
    lat: float
    alt: float
    speed: float
    type: str
    heading_in: float = 0.0
    heading_out: float = 0.0
    heading_change: float = 0.0
    gimbal_pitch: float = -90.0
    segment_idx: int = -1

class FlightPlan(BaseModel):
    plan_id: str
    flight_alt_agl: float
    flight_speed: float
    total_distance_m: float
    estimated_duration_s: float
    photo_spacing_m: float
    waypoint_mode: str
    waypoints: List[GenericWaypoint]
    has_gimbal: bool = True
    lens_type: str = "single"
    drone_enum: int = 68
    drone_sub_enum: int = 0
    payload_enum: int = 52

    def to_dict(self) -> Dict[str, Any]:
        return self.model_dump()
