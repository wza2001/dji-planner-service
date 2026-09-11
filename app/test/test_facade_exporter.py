import pytest
from app.core.planner.facade_planner import generate_facade_trajectory
from app.core.exporters.facade_dji_exporter import package_facade_kmz
import zipfile
import io

def test_facade_exporter():
    result = generate_facade_trajectory(
        center_lat=40.0, center_lon=-73.0,
        view_lat=40.001, view_lon=-73.0,
        building_radius=10.0, standoff_dist=20.0,
        fov_v_deg=30.0, fov_h_deg=45.0,
        overlap_v=0.7, overlap_h=0.8,
        base_alt=10.0, building_height=50.0
    )

    # rigid payload
    kmz_bytes = package_facade_kmz(
        plan=result, center_lat=40.0, center_lon=-73.0,
        drone_enum=103, payload_enum=65535, speed=5.0,
        has_gimbal=False, view_lat=40.001, view_lon=-73.0, l_step_m=5.0
    )

    with zipfile.ZipFile(io.BytesIO(kmz_bytes), 'r') as kmz:
        template = kmz.read('wpmz/template.kml').decode('utf-8')
        waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')

        # turn mode & curvature
        assert "toPointAndPassWithContinuityCurvature" in waylines
        assert "toPointAndStopWithDiscontinuityCurvature" in waylines

        # missing gimbal action for rigid payload
        assert "gimbalRotate" not in waylines
        assert "gimbalRotate" not in template

        # toward POI and multipleDistance
        assert "towardPOI" in template
        assert "towardPOI" in waylines
        assert "multipleDistance" in waylines

        # RC lost failsafe
        assert "goContinue" in template
