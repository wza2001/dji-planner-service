import math
from typing import List, Tuple
from app.models.facade import PointType, TriggerAction, FacadeWaypoint, FacadeTrajectoryResult

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate the bearing from point 1 to point 2 in degrees (0-360)."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)

    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    bearing_rad = math.atan2(y, x)
    return (math.degrees(bearing_rad) + 360) % 360

def calculate_destination(lat: float, lon: float, distance_m: float, bearing_deg: float) -> Tuple[float, float]:
    """Calculate the destination coordinates given a start point, distance, and bearing."""
    R = 6378137.0 # Earth radius in meters
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing_deg)

    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance_m / R) +
                            math.cos(lat_rad) * math.sin(distance_m / R) * math.cos(bearing_rad))
    new_lon_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance_m / R) * math.cos(lat_rad),
                                       math.cos(distance_m / R) - math.sin(lat_rad) * math.sin(new_lat_rad))
    return math.degrees(new_lat_rad), math.degrees(new_lon_rad)

def distance_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two coordinates in meters."""
    R = 6378137.0
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def generate_facade_trajectory(
    center_lat: float,
    center_lon: float,
    view_lat: float,
    view_lon: float,
    building_radius: float,
    standoff_dist: float,
    fov_v_deg: float,
    fov_h_deg: float,
    overlap_v: float,
    overlap_h: float,
    base_alt: float,
    building_height: float
) -> FacadeTrajectoryResult:
    # Calculate initial boresight bearing from center to viewpoint
    boresight_azimuth = calculate_bearing(center_lat, center_lon, view_lat, view_lon)

    # Radius of flight path
    R_flight = building_radius + standoff_dist

    # Step sizes
    delta_H = 2 * standoff_dist * math.tan(math.radians(fov_v_deg / 2)) * (1 - overlap_v)
    L_step = 2 * standoff_dist * math.tan(math.radians(fov_h_deg / 2)) * (1 - overlap_h)
    delta_theta = math.degrees(L_step / R_flight)

    # Total layers
    layers = max(1, math.ceil(building_height / delta_H))

    # Define angles
    angle_start = (boresight_azimuth - 95) % 360
    angle_end = (boresight_azimuth + 95) % 360
    total_sweep_angle = 190.0

    waypoints = []
    total_distance = 0.0
    prev_wp = None

    def add_waypoint(wp: FacadeWaypoint):
        nonlocal total_distance, prev_wp
        if prev_wp is not None:
            total_distance += distance_between(prev_wp.lat, prev_wp.lon, wp.lat, wp.lon)
            # Add altitude distance if it's a climb
            if wp.point_type == PointType.LADDER_CLIMB:
                total_distance += abs(wp.alt_wgs84 - prev_wp.alt_wgs84)
        waypoints.append(wp)
        prev_wp = wp

    # 1. Approach Waypoint
    approach_dist = R_flight + 15
    approach_lat, approach_lon = calculate_destination(center_lat, center_lon, approach_dist, boresight_azimuth)
    add_waypoint(FacadeWaypoint(
        lat=approach_lat, lon=approach_lon, alt_wgs84=base_alt + delta_H / 2,
        heading_deg=(boresight_azimuth + 180) % 360, point_type=PointType.APPROACH,
        trigger_action=TriggerAction.NONE
    ))

    current_alt = base_alt + delta_H / 2

    for i in range(layers):
        is_even = (i % 2 == 0)

        # Arc points
        arc_points = []
        num_arc_steps = max(2, math.ceil(total_sweep_angle / delta_theta)) + 1

        # Adjust delta_theta so it precisely covers total_sweep_angle
        actual_delta_theta = total_sweep_angle / (num_arc_steps - 1)

        for j in range(num_arc_steps):
            if is_even:
                # Clockwise: start to end
                bearing = (angle_start + j * actual_delta_theta) % 360
            else:
                # Counter-clockwise: end to start
                bearing = (angle_end - j * actual_delta_theta) % 360

            lat, lon = calculate_destination(center_lat, center_lon, R_flight, bearing)
            heading = (bearing + 180) % 360

            trigger = TriggerAction.NONE
            if j == 0:
                trigger = TriggerAction.START_SHOOT
            elif j == num_arc_steps - 1:
                trigger = TriggerAction.STOP_SHOOT

            arc_points.append(FacadeWaypoint(
                lat=lat, lon=lon, alt_wgs84=current_alt, heading_deg=heading,
                point_type=PointType.ARC_ACTIVE, trigger_action=trigger
            ))

        for wp in arc_points:
            add_waypoint(wp)

        # Turnaround Ladder (if not the last layer)
        if i < layers - 1:
            end_wp = arc_points[-1]
            end_bearing = (angle_end if is_even else angle_start) % 360

            # LADDER_OUT
            out_dist = R_flight + 5
            out_lat, out_lon = calculate_destination(center_lat, center_lon, out_dist, end_bearing)
            add_waypoint(FacadeWaypoint(
                lat=out_lat, lon=out_lon, alt_wgs84=current_alt,
                heading_deg=end_wp.heading_deg, point_type=PointType.LADDER_OUT,
                trigger_action=TriggerAction.NONE
            ))

            # LADDER_CLIMB
            next_alt = current_alt + delta_H
            add_waypoint(FacadeWaypoint(
                lat=out_lat, lon=out_lon, alt_wgs84=next_alt,
                heading_deg=end_wp.heading_deg, point_type=PointType.LADDER_CLIMB,
                trigger_action=TriggerAction.NONE
            ))

            # LADDER_IN (Start of next layer)
            in_lat, in_lon = calculate_destination(center_lat, center_lon, R_flight, end_bearing)
            add_waypoint(FacadeWaypoint(
                lat=in_lat, lon=in_lon, alt_wgs84=next_alt,
                heading_deg=end_wp.heading_deg, point_type=PointType.LADDER_IN,
                trigger_action=TriggerAction.NONE
            ))

            current_alt = next_alt

    # Exit Waypoint
    exit_dist = R_flight + 15
    exit_alt = current_alt + 15
    exit_lat, exit_lon = calculate_destination(center_lat, center_lon, exit_dist, boresight_azimuth)
    add_waypoint(FacadeWaypoint(
        lat=exit_lat, lon=exit_lon, alt_wgs84=exit_alt,
        heading_deg=(boresight_azimuth + 180) % 360, point_type=PointType.EXIT,
        trigger_action=TriggerAction.NONE
    ))

    return FacadeTrajectoryResult(
        layers=layers,
        total_distance=total_distance,
        waypoints=waypoints
    )
