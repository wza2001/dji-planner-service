import pytest
from app.core.planner.facade_planner import generate_facade_trajectory
from app.models.facade import PointType, TriggerAction

def test_facade_trajectory_generation():
    result = generate_facade_trajectory(
        center_lat=40.0,
        center_lon=-73.0,
        view_lat=40.001,
        view_lon=-73.0,
        building_radius=10.0,
        standoff_dist=20.0,
        fov_v_deg=30.0,
        fov_h_deg=45.0,
        overlap_v=0.7,
        overlap_h=0.8,
        base_alt=10.0,
        building_height=50.0
    )

    assert result.layers > 0
    assert len(result.waypoints) > 0

    # Approach waypoint check
    assert result.waypoints[0].point_type == PointType.APPROACH

    # Exit waypoint check
    assert result.waypoints[-1].point_type == PointType.EXIT

    # Arc points check
    arc_points = [wp for wp in result.waypoints if wp.point_type == PointType.ARC_ACTIVE]
    assert len(arc_points) > 0

    # Ladder points check
    ladder_out = [wp for wp in result.waypoints if wp.point_type == PointType.LADDER_OUT]
    ladder_climb = [wp for wp in result.waypoints if wp.point_type == PointType.LADDER_CLIMB]
    ladder_in = [wp for wp in result.waypoints if wp.point_type == PointType.LADDER_IN]

    if result.layers > 1:
        assert len(ladder_out) == result.layers - 1
        assert len(ladder_climb) == result.layers - 1
        assert len(ladder_in) == result.layers - 1
