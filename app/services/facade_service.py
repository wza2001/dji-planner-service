import io
import zipfile
import json
from shapely.geometry import shape
from shapely.wkt import loads as wkt_loads
from typing import Optional, Dict, Any, Tuple
from app.core.planner.facade_planner import generate_facade_trajectory, calculate_destination, calculate_bearing
from app.core.exporters.facade_dji_exporter import package_facade_kmz
import math

def calculate_symmetric_viewpoint(center_lat: float, center_lon: float, view_lat: float, view_lon: float) -> Tuple[float, float]:
    bearing_from_center = calculate_bearing(center_lat, center_lon, view_lat, view_lon)
    dist = math.sqrt((view_lat - center_lat)**2 + (view_lon - center_lon)**2) * 111320 # approximate deg to m
    opposite_bearing = (bearing_from_center + 180) % 360
    return calculate_destination(center_lat, center_lon, dist, opposite_bearing)

def orchestrate_facade_missions(
    geometry_data: Any,
    ground_alt_m: float,
    building_height_m: float,
    viewpoint_a: Dict[str, float],
    viewpoint_b: Optional[Dict[str, float]],
    drone_model: str,
    camera_model: str,
    face_gsd_cm: float,
    drone_enum: int,
    payload_enum: int,
    has_gimbal: bool,
    fov_v_deg: float,
    fov_h_deg: float,
    focal_length_mm: float
) -> Tuple[bytes, Dict[str, Any]]:

    # 1. Parse geometry to find center and radius
    if isinstance(geometry_data, dict):
        geom = shape(geometry_data)
    else:
        geom = wkt_loads(geometry_data)

    centroid = geom.centroid
    center_lon, center_lat = centroid.x, centroid.y

    # Approx building radius from center to farthest point of geometry
    # A simple circle that encompasses the building
    coords = list(geom.exterior.coords)
    max_dist = 0.0
    for lon, lat in coords:
        dist = math.sqrt((lon - center_lon)**2 + (lat - center_lat)**2) * 111320 # rough approx in meters
        if dist > max_dist:
            max_dist = dist
    building_radius = max_dist

    # Viewpoint B
    if not viewpoint_b:
        opp_lat, opp_lon = calculate_symmetric_viewpoint(center_lat, center_lon, viewpoint_a['lat'], viewpoint_a['lon'])
        viewpoint_b = {'lat': opp_lat, 'lon': opp_lon}

    # Calculate standoff from GSD
    # GSD = (sensor_width / image_width) * (dist / focal_length)
    # This is a bit complex without sensor details, we'll use a fixed formula or assume a standoff
    # for simplicity in this implementation if not fully provided.
    standoff_dist = 30.0 # Placeholder if we don't have full camera math here, or compute:
    # We can refine if we have pixel size. For now use a reasonable default.

    # We can also compute L_step to pass to export
    overlap_v = 0.7
    overlap_h = 0.8
    L_step = 2 * standoff_dist * math.tan(math.radians(fov_h_deg / 2)) * (1 - overlap_h)

    speed = 5.0

    # Mission A
    traj_A = generate_facade_trajectory(
        center_lat=center_lat, center_lon=center_lon,
        view_lat=viewpoint_a['lat'], view_lon=viewpoint_a['lon'],
        building_radius=building_radius, standoff_dist=standoff_dist,
        fov_v_deg=fov_v_deg, fov_h_deg=fov_h_deg,
        overlap_v=overlap_v, overlap_h=overlap_h,
        base_alt=ground_alt_m, building_height=building_height_m
    )

    kmz_A_bytes = package_facade_kmz(
        plan=traj_A, center_lat=center_lat, center_lon=center_lon,
        drone_enum=drone_enum, payload_enum=payload_enum, speed=speed,
        has_gimbal=has_gimbal, view_lat=viewpoint_a['lat'], view_lon=viewpoint_a['lon'],
        l_step_m=L_step
    )

    # Mission B
    traj_B = generate_facade_trajectory(
        center_lat=center_lat, center_lon=center_lon,
        view_lat=viewpoint_b['lat'], view_lon=viewpoint_b['lon'],
        building_radius=building_radius, standoff_dist=standoff_dist,
        fov_v_deg=fov_v_deg, fov_h_deg=fov_h_deg,
        overlap_v=overlap_v, overlap_h=overlap_h,
        base_alt=ground_alt_m, building_height=building_height_m
    )

    kmz_B_bytes = package_facade_kmz(
        plan=traj_B, center_lat=center_lat, center_lon=center_lon,
        drone_enum=drone_enum, payload_enum=payload_enum, speed=speed,
        has_gimbal=has_gimbal, view_lat=viewpoint_b['lat'], view_lon=viewpoint_b['lon'],
        l_step_m=L_step
    )

    summary = {
        "building_height_m": building_height_m,
        "face_gsd_cm": face_gsd_cm,
        "layers": traj_A.layers,
        "standoff_distance_m": standoff_dist,
        "mission_A": {
            "total_distance_m": traj_A.total_distance,
            "waypoints": len(traj_A.waypoints)
        },
        "mission_B": {
            "total_distance_m": traj_B.total_distance,
            "waypoints": len(traj_B.waypoints)
        }
    }

    mem_zip = io.BytesIO()
    with zipfile.ZipFile(mem_zip, 'w', zipfile.ZIP_DEFLATED) as main_zip:
        main_zip.writestr('Face_A.kmz', kmz_A_bytes)
        main_zip.writestr('Face_B.kmz', kmz_B_bytes)
        main_zip.writestr('mission_summary.json', json.dumps(summary, indent=2))

    return mem_zip.getvalue(), summary
