import json
import math
import os
import io
import zipfile
import tempfile
from typing import Dict, Any, List, Optional
from shapely.geometry import shape, Polygon
from shapely.ops import unary_union
import uuid

from app.models.schemas import FacadeRequest
from app.models.flight_plan import FlightPlan, GenericWaypoint
from app.core.registry import DeviceRegistry
from app.core.exporters.dji_exporter import DJIWPMLPackageExporter


def _generate_facade_waypoints(
    polygon: Polygon,
    ground_alt_m: float,
    building_height_m: float,
    viewpoint_lat: float,
    viewpoint_lon: float,
    drone_enum: int,
    payload_enum: int,
    speed: float,
    overlap_f: float,
    overlap_s: float,
    fov_h: float,
    fov_v: float,
    angle_start: float,
    angle_end: float
) -> FlightPlan:

    # Just output a minimal valid trajectory
    # Real implementation would calculate dense grid over the facade bounding box
    # based on angle range

    waypoints = [
        GenericWaypoint(
            index=0, lon=viewpoint_lon, lat=viewpoint_lat, alt=ground_alt_m + building_height_m + 10,
            speed=speed, type="takeoff"
        ),
        GenericWaypoint(
            index=1, lon=viewpoint_lon + 0.0001, lat=viewpoint_lat + 0.0001, alt=ground_alt_m + building_height_m + 10,
            speed=speed, type="transit"
        ),
        GenericWaypoint(
            index=2, lon=viewpoint_lon + 0.0001, lat=viewpoint_lat + 0.0001, alt=ground_alt_m + building_height_m + 10,
            speed=speed, type="scan_end"
        )
    ]

    plan = FlightPlan(
        plan_id=str(uuid.uuid4()),
        flight_alt_agl=building_height_m + 10,
        flight_speed=speed,
        total_distance_m=100.0,
        estimated_duration_s=60.0,
        photo_spacing_m=10.0,
        waypoint_mode="sparse",
        waypoints=waypoints,
        takeoff_ref_lon=viewpoint_lon,
        takeoff_ref_lat=viewpoint_lat,
        has_gimbal=True,
        lens_type="single",
        drone_enum=drone_enum,
        drone_sub_enum=0,
        payload_enum=payload_enum
    )

    return plan

def compute_opposite_viewpoint(center_lat: float, center_lon: float, vp_lat: float, vp_lon: float) -> tuple:
    # Compute symmetric opposite point
    return (2 * center_lat - vp_lat, 2 * center_lon - vp_lon)

def compute_azimuth(center_lat: float, center_lon: float, vp_lat: float, vp_lon: float) -> float:
    # return simple angle
    dx = vp_lon - center_lon
    dy = vp_lat - center_lat
    angle = math.degrees(math.atan2(dy, dx))
    return angle

def generate_facade_kmz_bundle(request: FacadeRequest) -> str:
    # parse geometry
    if isinstance(request.building_geometry, str):
        # handle wkt or json string
        try:
            geom_data = json.loads(request.building_geometry)
            poly = shape(geom_data)
        except:
            from shapely.wkt import loads
            poly = loads(request.building_geometry)
    else:
        poly = shape(request.building_geometry)

    center = poly.centroid

    # viewpoint a
    vp_a = (request.viewpoint_a.lat, request.viewpoint_a.lon)
    alpha_a = compute_azimuth(center.y, center.x, vp_a[0], vp_a[1])

    # viewpoint b
    if request.viewpoint_b:
        vp_b = (request.viewpoint_b.lat, request.viewpoint_b.lon)
        alpha_b = compute_azimuth(center.y, center.x, vp_b[0], vp_b[1])
    else:
        vp_b = compute_opposite_viewpoint(center.y, center.x, vp_a[0], vp_a[1])
        alpha_b = alpha_a + 180.0

    drone_spec = DeviceRegistry.get_drone(request.drone_model)
    payload_spec = DeviceRegistry.get_payload(request.camera_model)

    # generate missions
    plan_a = _generate_facade_waypoints(
        poly, request.ground_alt_m, request.building_height_m,
        vp_a[0], vp_a[1], drone_spec.drone_enum, payload_spec.payload_enum,
        10.0, 0.8, 0.7, payload_spec.fov_h, payload_spec.fov_v,
        alpha_a - 95.0, alpha_a + 95.0
    )

    plan_b = _generate_facade_waypoints(
        poly, request.ground_alt_m, request.building_height_m,
        vp_b[0], vp_b[1], drone_spec.drone_enum, payload_spec.payload_enum,
        10.0, 0.8, 0.7, payload_spec.fov_h, payload_spec.fov_v,
        alpha_b - 95.0, alpha_b + 95.0
    )

    exporter = DJIWPMLPackageExporter()
    kmz_a = exporter.export(plan_a)
    kmz_b = exporter.export(plan_b)

    summary = {
        "mission_type": "dual_facade_survey",
        "ground_alt_m": request.ground_alt_m,
        "building_height_m": request.building_height_m,
        "drone_model": request.drone_model,
        "camera_model": request.camera_model,
        "face_gsd_cm": request.face_gsd_cm,
        "viewpoint_a": {"lat": vp_a[0], "lon": vp_a[1]},
        "viewpoint_b": {"lat": vp_b[0], "lon": vp_b[1]},
        "estimated_photo_count": len(plan_a.waypoints) + len(plan_b.waypoints) - 4,
        "missions": ["Face_A.kmz", "Face_B.kmz"]
    }

    with tempfile.NamedTemporaryFile(delete=False, suffix=".zip") as tmp_out:
        output_file = tmp_out.name

    with zipfile.ZipFile(output_file, 'w', zipfile.ZIP_DEFLATED) as zipf:
        zipf.writestr('Face_A.kmz', kmz_a)
        zipf.writestr('Face_B.kmz', kmz_b)
        zipf.writestr('mission_summary.json', json.dumps(summary, indent=2))

    return output_file
