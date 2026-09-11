import math
import pytest
from app.core.planner.facade_planner import FacadePlanner
from app.models.facade import PointType, TriggerAction

def test_facade_geometry_basic():
    planner = FacadePlanner(
        center_lat=0.0, center_lon=0.0,
        view_lat=0.01, view_lon=0.01,
        standoff_m=20.0,
        building_radius_m=10.0,
        fov_h_deg=90.0, fov_v_deg=90.0,
        overlap_h=0.0, overlap_v=0.0,
        z_start=10.0
    )

    azimuth = planner.calculate_boresight_azimuth()
    assert azimuth == pytest.approx(45.0, 0.1)

    delta_h, delta_theta = planner.get_layer_params()
    # D = 20
    # delta_h = 2 * 20 * tan(45) * 1 = 40
    # L_step = 2 * 20 * tan(45) * 1 = 40
    # delta_theta = L_step / R = 40 / 30 = 4/3 rad = 76.39 deg
    assert delta_h == pytest.approx(40.0, 0.1)
    assert delta_theta == pytest.approx(math.degrees(40.0 / 30.0), 0.1)

    result = planner.generate_trajectory(num_layers=2)
    assert result.layers == 2

    wps = result.waypoints
    assert wps[0].point_type == PointType.APPROACH
    assert wps[-1].point_type == PointType.EXIT

    # Verify sweep clamp (azimuth 45)
    # Layer 0 starts at 45-95 = -50 = 310
    # and goes to 45+95 = 140
    arc_points_layer_0 = [wp for wp in wps if wp.point_type == PointType.ARC_ACTIVE and wp.alt_wgs84 == 10.0]
    assert len(arc_points_layer_0) > 0
    assert arc_points_layer_0[0].trigger_action == TriggerAction.START_SHOOT
    assert arc_points_layer_0[-1].trigger_action == TriggerAction.STOP_SHOOT

    start_heading = arc_points_layer_0[0].heading_deg
    end_heading = arc_points_layer_0[-1].heading_deg

    # headings are (ang + 180) % 360
    # expected start ang = 310 -> heading = 310 + 180 = 490 % 360 = 130
    # expected end ang = 140 -> heading = 140 + 180 = 320
    assert start_heading == pytest.approx(130.0, 0.1)
    assert end_heading == pytest.approx(320.0, 0.1)

    ladder_out = [wp for wp in wps if wp.point_type == PointType.LADDER_OUT]
    assert len(ladder_out) == 1

    ladder_climb = [wp for wp in wps if wp.point_type == PointType.LADDER_CLIMB]
    assert len(ladder_climb) == 1
    assert ladder_climb[0].alt_wgs84 == pytest.approx(50.0, 0.1) # 10 + 40

    ladder_in = [wp for wp in wps if wp.point_type == PointType.LADDER_IN]
    assert len(ladder_in) == 1

    arc_points_layer_1 = [wp for wp in wps if wp.point_type == PointType.ARC_ACTIVE and wp.alt_wgs84 == pytest.approx(50.0, 0.1)]
    start_heading_l1 = arc_points_layer_1[0].heading_deg
    end_heading_l1 = arc_points_layer_1[-1].heading_deg

    assert start_heading_l1 == pytest.approx(320.0, 0.1)
    assert end_heading_l1 == pytest.approx(130.0, 0.1)

    assert result.total_distance > 0
