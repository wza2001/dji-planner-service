from pydantic import BaseModel, Field
from typing import Union, Dict, Any

class PlannerRequest(BaseModel):
    fov_h: float = Field(default=84.0, description="Horizontal FOV in degrees")
    fov_v: float = Field(default=60.0, description="Vertical FOV in degrees")
    agl: float = Field(default=240.0, description="Flight altitude AGL in meters")
    speed: float = Field(default=10.0, description="Flight speed in m/s")
    overlap_f: float = Field(default=0.80, description="Forward overlap ratio")
    overlap_s: float = Field(default=0.70, description="Side overlap ratio")
    drone_enum: int = Field(default=95, description="Drone model enum value")
    payload_enum: int = Field(default=52, description="Payload enum value")
    waypoint_mode: str = Field(default="sparse", description="Waypoint mode ('sparse' or 'dense')")
    boundary_geojson: Union[Dict[str, Any], str] = Field(..., description="GeoJSON polygon coordinates for flight boundary")
