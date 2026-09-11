from pydantic import BaseModel
from enum import Enum
from typing import List

class PointType(Enum):
    APPROACH = "APPROACH"
    ARC_ACTIVE = "ARC_ACTIVE"
    LADDER_OUT = "LADDER_OUT"
    LADDER_CLIMB = "LADDER_CLIMB"
    LADDER_IN = "LADDER_IN"
    EXIT = "EXIT"

class TriggerAction(Enum):
    START_SHOOT = "START_SHOOT"
    STOP_SHOOT = "STOP_SHOOT"
    NONE = "NONE"

class FacadeWaypoint(BaseModel):
    lat: float
    lon: float
    alt_wgs84: float
    heading_deg: float
    point_type: PointType
    trigger_action: TriggerAction

class FacadeTrajectoryResult(BaseModel):
    layers: int
    total_distance: float
    waypoints: List[FacadeWaypoint]
