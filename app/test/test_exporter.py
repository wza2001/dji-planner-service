import pytest
from app.models.flight_plan import FlightPlan, GenericWaypoint
from app.core.exporters.dji_exporter import DJIWPMLPackageExporter
import zipfile
import io

def create_mock_flight_plan(has_gimbal):
    wp = GenericWaypoint(
        index=0,
        lon=12.0,
        lat=34.0,
        alt=100.0,
        speed=10.0,
        type='scan_start'
    )
    return FlightPlan(
        plan_id="test_plan",
        flight_alt_agl=100.0,
        flight_speed=10.0,
        total_distance_m=1000.0,
        estimated_duration_s=100.0,
        photo_spacing_m=20.0,
        waypoint_mode="sparse",
        waypoints=[wp],
        has_gimbal=has_gimbal,
        lens_type="single",
        drone_enum=89,
        drone_sub_enum=1,
        payload_enum=52
    )

def test_exporter_with_gimbal():
    plan = create_mock_flight_plan(has_gimbal=True)
    exporter = DJIWPMLPackageExporter()
    kmz_bytes = exporter.export(plan)

    with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as kmz:
        waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')
        assert 'gimbalRotate' in waylines

def test_exporter_without_gimbal():
    plan = create_mock_flight_plan(has_gimbal=False)
    exporter = DJIWPMLPackageExporter()
    kmz_bytes = exporter.export(plan)

    with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as kmz:
        waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')
        assert 'gimbalRotate' not in waylines
