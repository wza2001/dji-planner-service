import math
from typing import List
from app.models.facade import FacadeWaypoint, PointType, TriggerAction, FacadeTrajectoryResult
from app.utils.geo_geometry import get_meter_degree_factors

class FacadePlanner:
    def __init__(self,
                 center_lat: float, center_lon: float,
                 view_lat: float, view_lon: float,
                 face_gsd: float = None, # Standoff distance or gsd could be used
                 standoff_m: float = None,
                 building_radius_m: float = 0.0,
                 fov_h_deg: float = 84.0, fov_v_deg: float = 60.0,
                 overlap_h: float = 0.8, overlap_v: float = 0.8,
                 z_start: float = 10.0):

        self.center_lat = center_lat
        self.center_lon = center_lon
        self.view_lat = view_lat
        self.view_lon = view_lon

        self.standoff_m = standoff_m if standoff_m is not None else 20.0
        self.building_radius_m = building_radius_m
        self.radius_m = self.building_radius_m + self.standoff_m

        self.fov_h = math.radians(fov_h_deg)
        self.fov_v = math.radians(fov_v_deg)
        self.overlap_h = overlap_h
        self.overlap_v = overlap_v

        self.z_start = z_start

    def calculate_boresight_azimuth(self) -> float:
        """
        Given the building center (X_center, Y_center) and a ground station / view reference coordinate (X_view, Y_view),
        derive the boresight azimuth \alpha_0.
        """
        # We need azimuth from center to view.
        m_lat, m_lon = get_meter_degree_factors(self.center_lat)

        dy = (self.view_lat - self.center_lat) * m_lat
        dx = (self.view_lon - self.center_lon) * m_lon

        # Calculate azimuth in degrees
        # Azimuth is clockwise from North (Y axis)
        # atan2(y, x) -> standard math coords
        # atan2(dx, dy) -> clockwise from North
        azimuth = math.degrees(math.atan2(dx, dy)) % 360
        return azimuth

    def _offset_lat_lon(self, lat: float, lon: float, offset_x_m: float, offset_y_m: float) -> tuple[float, float]:
        m_lat, m_lon = get_meter_degree_factors(lat)
        new_lat = lat + (offset_y_m / m_lat)
        new_lon = lon + (offset_x_m / m_lon)
        return new_lat, new_lon

    def get_layer_params(self):
        # Compute vertical layer step \Delta H
        # \Delta H = 2 \cdot D_standoff \cdot \tan(\frac{FOV_v}{2}) \cdot (1 - overlap_v)
        delta_h = 2 * self.standoff_m * math.tan(self.fov_v / 2) * (1 - self.overlap_v)

        # Compute horizontal arc step L_step and angular step \Delta \theta
        # L_step = 2 \cdot D_standoff \cdot \tan(\frac{FOV_h}{2}) \cdot (1 - overlap_h)
        # \Delta \theta = \frac{L_step}{R}
        l_step = 2 * self.standoff_m * math.tan(self.fov_h / 2) * (1 - self.overlap_h)
        delta_theta_rad = l_step / self.radius_m
        delta_theta = math.degrees(delta_theta_rad)

        return delta_h, delta_theta

    def _azimuth_to_offset(self, azimuth_deg: float, radius: float) -> tuple[float, float]:
        azimuth_rad = math.radians(azimuth_deg)
        # dx is E, dy is N
        dx = radius * math.sin(azimuth_rad)
        dy = radius * math.cos(azimuth_rad)
        return dx, dy

    def generate_trajectory(self, num_layers: int = 3) -> FacadeTrajectoryResult:
        waypoints = []
        total_distance = 0.0

        alpha_0 = self.calculate_boresight_azimuth()

        # Valid horizontal sweep angle to [alpha_0 - 95, alpha_0 + 95]
        start_angle = (alpha_0 - 95) % 360
        end_angle = (alpha_0 + 95) % 360

        delta_h, delta_theta = self.get_layer_params()

        # Approach Waypoint
        app_radius = self.radius_m + 15.0
        dx, dy = self._azimuth_to_offset(alpha_0, app_radius)
        app_lat, app_lon = self._offset_lat_lon(self.center_lat, self.center_lon, dx, dy)
        heading_app = (alpha_0 + 180) % 360 # pointing towards center

        waypoints.append(FacadeWaypoint(
            lat=app_lat,
            lon=app_lon,
            alt_wgs84=self.z_start,
            heading_deg=heading_app,
            point_type=PointType.APPROACH,
            trigger_action=TriggerAction.NONE
        ))

        current_z = self.z_start
        last_x, last_y = dx, dy
        last_z = current_z

        ladder_push_out_m = 4.0

        for k in range(num_layers):
            is_even = (k % 2 == 0)

            # Sweep span
            if is_even:
                # Clockwise from Point A (alpha_0 - 95) to Point B (alpha_0 + 95)
                current_angle = alpha_0 - 95
                target_angle = alpha_0 + 95
                step_sign = 1
            else:
                # Counter-clockwise from Point B to Point A
                current_angle = alpha_0 + 95
                target_angle = alpha_0 - 95
                step_sign = -1

            sweep_angles = []
            steps = int(190 / delta_theta)
            for i in range(steps + 1):
                sweep_angles.append(current_angle + step_sign * i * delta_theta)
            # Ensure target angle is included
            sweep_angles.append(target_angle)

            for i, ang in enumerate(sweep_angles):
                dx, dy = self._azimuth_to_offset(ang, self.radius_m)
                wp_lat, wp_lon = self._offset_lat_lon(self.center_lat, self.center_lon, dx, dy)
                # drone points to center
                heading = (ang + 180) % 360

                trigger = TriggerAction.NONE
                if i == 0:
                    trigger = TriggerAction.START_SHOOT
                elif i == len(sweep_angles) - 1:
                    trigger = TriggerAction.STOP_SHOOT

                # accumulate distance
                total_distance += math.sqrt((dx - last_x)**2 + (dy - last_y)**2 + (current_z - last_z)**2)
                last_x, last_y, last_z = dx, dy, current_z

                waypoints.append(FacadeWaypoint(
                    lat=wp_lat, lon=wp_lon, alt_wgs84=current_z, heading_deg=heading,
                    point_type=PointType.ARC_ACTIVE, trigger_action=trigger
                ))

            # 3-Stage Ladder Turnaround
            if k < num_layers - 1:
                # Radial Push-out
                po_dx, po_dy = self._azimuth_to_offset(target_angle, self.radius_m + ladder_push_out_m)
                po_lat, po_lon = self._offset_lat_lon(self.center_lat, self.center_lon, po_dx, po_dy)

                total_distance += math.sqrt((po_dx - last_x)**2 + (po_dy - last_y)**2 + (current_z - last_z)**2)
                last_x, last_y, last_z = po_dx, po_dy, current_z

                waypoints.append(FacadeWaypoint(
                    lat=po_lat, lon=po_lon, alt_wgs84=current_z, heading_deg=(target_angle + 180) % 360,
                    point_type=PointType.LADDER_OUT, trigger_action=TriggerAction.NONE
                ))

                # Vertical Climb
                current_z += delta_h
                total_distance += math.sqrt((po_dx - last_x)**2 + (po_dy - last_y)**2 + (current_z - last_z)**2)
                last_x, last_y, last_z = po_dx, po_dy, current_z

                waypoints.append(FacadeWaypoint(
                    lat=po_lat, lon=po_lon, alt_wgs84=current_z, heading_deg=(target_angle + 180) % 360,
                    point_type=PointType.LADDER_CLIMB, trigger_action=TriggerAction.NONE
                ))

                # Cut-in
                ci_dx, ci_dy = self._azimuth_to_offset(target_angle, self.radius_m)
                ci_lat, ci_lon = self._offset_lat_lon(self.center_lat, self.center_lon, ci_dx, ci_dy)

                total_distance += math.sqrt((ci_dx - last_x)**2 + (ci_dy - last_y)**2 + (current_z - last_z)**2)
                last_x, last_y, last_z = ci_dx, ci_dy, current_z

                waypoints.append(FacadeWaypoint(
                    lat=ci_lat, lon=ci_lon, alt_wgs84=current_z, heading_deg=(target_angle + 180) % 360,
                    point_type=PointType.LADDER_IN, trigger_action=TriggerAction.NONE
                ))

        # Exit Waypoint
        z_top = current_z
        exit_radius = self.radius_m + 15.0
        ex_dx, ex_dy = self._azimuth_to_offset(alpha_0, exit_radius)
        ex_lat, ex_lon = self._offset_lat_lon(self.center_lat, self.center_lon, ex_dx, ex_dy)

        total_distance += math.sqrt((ex_dx - last_x)**2 + (ex_dy - last_y)**2 + (z_top + 15.0 - last_z)**2)

        waypoints.append(FacadeWaypoint(
            lat=ex_lat, lon=ex_lon, alt_wgs84=z_top + 15.0, heading_deg=(alpha_0 + 180) % 360,
            point_type=PointType.EXIT, trigger_action=TriggerAction.NONE
        ))

        return FacadeTrajectoryResult(
            layers=num_layers,
            total_distance=total_distance,
            waypoints=waypoints
        )
