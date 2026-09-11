from pydantic import BaseModel, Field, model_validator
from typing import Union, Dict, Any, Optional

class PlannerRequest(BaseModel):
    drone_model: Optional[str] = Field(default="matrice_350_rtk", description="Drone model preset identifier")
    camera_model: Optional[str] = Field(default="zenmuse_p1_35mm", description="Camera model preset identifier")

    is_custom_camera: bool = Field(default=False, description="Flag for custom 3rd-party camera")
    sensor_width_mm: Optional[float] = Field(default=None, description="Sensor width in mm")
    sensor_height_mm: Optional[float] = Field(default=None, description="Sensor height in mm")
    focal_length_mm: Optional[float] = Field(default=None, description="Focal length in mm")
    has_gimbal: Optional[bool] = Field(default=True, description="Whether payload has a gimbal")
    lens_type: Optional[str] = Field(default="single", description="Lens type")

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

    @model_validator(mode='after')
    def validate_custom_camera(self) -> 'PlannerRequest':
        if self.is_custom_camera:
            if self.sensor_width_mm is None or self.sensor_height_mm is None or self.focal_length_mm is None:
                raise ValueError("sensor_width_mm, sensor_height_mm, and focal_length_mm are required when is_custom_camera is True")
        return self
