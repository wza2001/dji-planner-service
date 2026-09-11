import pytest
from app.models.flight_plan import FlightPlan, GenericWaypoint
from app.core.exporters.dji_exporter import DJIWPMLPackageExporter
import zipfile
import io

def create_mock_facade_plan():
    wps = [
        GenericWaypoint(index=0, lon=12.0, lat=34.0, alt=100.0, speed=10.0, type='APPROACH', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0),
        GenericWaypoint(index=1, lon=12.0, lat=34.0, alt=100.0, speed=10.0, type='ARC_ACTIVE', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0, gimbal_pitch=-30.0),
        GenericWaypoint(index=2, lon=12.0, lat=34.0, alt=100.0, speed=10.0, type='ARC_ACTIVE', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0, gimbal_pitch=-30.0),
        GenericWaypoint(index=3, lon=12.0, lat=34.0, alt=100.0, speed=10.0, type='LADDER_OUT', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0),
        GenericWaypoint(index=4, lon=12.0, lat=34.0, alt=110.0, speed=10.0, type='LADDER_CLIMB', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0),
        GenericWaypoint(index=5, lon=12.0, lat=34.0, alt=110.0, speed=10.0, type='LADDER_IN', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0),
        GenericWaypoint(index=6, lon=12.0, lat=34.0, alt=110.0, speed=10.0, type='ARC_ACTIVE', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0, gimbal_pitch=-30.0),
        GenericWaypoint(index=7, lon=12.0, lat=34.0, alt=110.0, speed=10.0, type='ARC_ACTIVE', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0, gimbal_pitch=-30.0),
        GenericWaypoint(index=8, lon=12.0, lat=34.0, alt=110.0, speed=10.0, type='EXIT', poi_lon=12.0, poi_lat=34.0, poi_alt=100.0)
    ]
    return FlightPlan(
        plan_id="facade_plan",
        flight_alt_agl=100.0,
        flight_speed=5.0,
        total_distance_m=1000.0,
        estimated_duration_s=200.0,
        photo_spacing_m=10.0,
        waypoint_mode="sparse",
        waypoints=wps,
        has_gimbal=True,
        lens_type="single",
        drone_enum=68,
        drone_sub_enum=0,
        payload_enum=52
    )

def test_facade_wpml():
    plan = create_mock_facade_plan()
    exporter = DJIWPMLPackageExporter()
    kmz_bytes = exporter.export(plan)

    with zipfile.ZipFile(io.BytesIO(kmz_bytes)) as kmz:
        template = kmz.read('wpmz/template.kml').decode('utf-8')
        waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')

        # Check safety failsafes
        assert '<wpml:executeHeightMode>WGS84</wpml:executeHeightMode>' in template
        assert '<wpml:executeHeightMode>WGS84</wpml:executeHeightMode>' in waylines
        assert '<wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>' in template
        assert '<wpml:finishAction>goHome</wpml:finishAction>' in template

        # Check turn modes and straight lines
        assert 'toPointAndPassWithContinuityCurvature' in waylines
        assert 'toPointAndStopWithDiscontinuityCurvature' in waylines
        assert 'towardPOI' in waylines
        assert '<wpml:useStraightLine>1</wpml:useStraightLine>' in waylines

        # Check shutter action groups
        assert 'multipleDistance' in waylines
        assert 'stopShooting' in waylines

if __name__ == "__main__":
    pytest.main(["-v", "test_facade_exporter.py"])
