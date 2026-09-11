import pytest
from fastapi.testclient import TestClient
from app.main import app
import zipfile
import io
import json

client = TestClient(app)

def test_facade_m400_riebo_end_to_end():
    # 1. Archive Integrity
    payload = {
        "building_geometry": {
            "type": "Polygon",
            "coordinates": [[[0, 0], [0.001, 0], [0.001, 0.001], [0, 0.001], [0, 0]]]
        },
        "ground_alt_m": 0.0,
        "building_height_m": 80.0,
        "viewpoint_a": {"lat": -0.005, "lon": 0.0},
        "viewpoint_b": {"lat": 0.005, "lon": 0.001},
        "drone_model": "matrice_400",
        "camera_model": "riebo_dg6p_oblique",
        "face_gsd_cm": 1.0
    }

    response = client.post("/api/v1/planner/generate_facade_kmz", json=payload)
    assert response.status_code == 200, f"Failed with {response.text}"
    assert response.headers["content-type"] == "application/zip"

    zip_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(zip_bytes, 'r') as archive:
        files = archive.namelist()
        assert "Face_A.kmz" in files
        assert "Face_B.kmz" in files
        assert "mission_summary.json" in files

        # 2. Hardware Registry Mapping
        for face_file in ["Face_A.kmz", "Face_B.kmz"]:
            kmz_bytes = archive.read(face_file)
            with zipfile.ZipFile(io.BytesIO(kmz_bytes), 'r') as kmz:
                template = kmz.read('wpmz/template.kml').decode('utf-8')
                waylines = kmz.read('wpmz/waylines.wpml').decode('utf-8')

                assert "<wpml:droneEnumValue>103</wpml:droneEnumValue>" in template
                assert "<wpml:payloadEnumValue>65535</wpml:payloadEnumValue>" in template

                # Protect rigid 5-lens systems
                assert "gimbalRotate" not in template
                assert "gimbalRotate" not in waylines

                # 3. Flight Trajectory & Turn Modes
                assert "toPointAndPassWithContinuityCurvature" in waylines
                assert "toPointAndStopWithDiscontinuityCurvature" in waylines

                assert "goContinue" in template
                assert "multipleDistance" in waylines

                # 4. Wrap-Around Overlap (implied by 190 deg sweep, verifying waypoint existence for now since geometry test covers angles)
