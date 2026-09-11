import pytest
import io
import zipfile
import math
from fastapi.testclient import TestClient

from app.models.hardware import PayloadSpec, LensType
from app.core.registry import DeviceRegistry
from app.main import app

client = TestClient(app)

def test_registry_calculations():
    # Verify FOV calculation for standard full-frame 35mm lens (35.9 x 24.0 mm sensor, 35mm focal length)
    # equals approx 54.3 deg horizontal and 37.8 deg vertical
    payload = DeviceRegistry.build_custom_payload(
        name="test_35mm",
        sensor_width_mm=35.9,
        sensor_height_mm=24.0,
        focal_length_mm=35.0
    )

    fov_h = payload.fov_h
    fov_v = payload.fov_v

    assert math.isclose(fov_h, 54.3, rel_tol=1e-2), f"Expected FOV H ~ 54.3, got {fov_h}"
    assert math.isclose(fov_v, 37.8, rel_tol=1e-2), f"Expected FOV V ~ 37.8, got {fov_v}"

def test_fixed_mount_no_gimbal_export():
    import json

    # 1. Prepare minimal request
    geojson = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[[0, 0], [0.001, 0], [0.001, 0.001], [0, 0.001], [0, 0]]]
                }
            }
        ]
    }

    # We want to test 'share_102s_oblique' which has `has_gimbal=False`.
    request_data = {
        "boundary_geojson": geojson,
        "drone_model": "matrice_350_rtk",
        "camera_model": "share_102s_oblique",
        "agl": 120,
        "speed": 10,
        "overlap_f": 0.8,
        "overlap_s": 0.7,
        "is_custom_camera": False
    }

    response = client.post("/api/v1/planner/generate_kmz", json=request_data)
    assert response.status_code == 200

    # Unzip KMZ
    kmz_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(kmz_bytes, 'r') as kmz:
        waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')

    # Assert
    assert "<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>" not in waylines, "Gimbal action shouldn't be present"
    assert "<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>" in waylines, "Follow wayline mode should be present"

def test_custom_camera_flow():
    import json

    geojson = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[[0, 0], [0.001, 0], [0.001, 0.001], [0, 0.001], [0, 0]]]
                }
            }
        ]
    }

    # Custom Phase One specs: 53.4 x 40.0 mm, 50mm focal length
    request_data = {
        "boundary_geojson": geojson,
        "drone_model": "matrice_350_rtk",
        "is_custom_camera": True,
        "sensor_width_mm": 53.4,
        "sensor_height_mm": 40.0,
        "focal_length_mm": 50.0,
        "has_gimbal": True,
        "lens_type": "single",
        "agl": 120,
        "speed": 10,
        "overlap_f": 0.8,
        "overlap_s": 0.7
    }

    response = client.post("/api/v1/planner/generate_kmz", json=request_data)
    assert response.status_code == 200, f"Error: {response.text}"

    # Unzip and assert KMZ structure is valid
    kmz_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(kmz_bytes, 'r') as kmz:
        assert 'wpmz/template.kml' in kmz.namelist()
        assert 'wpmz/waylines.wpml' in kmz.namelist()
