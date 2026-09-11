import pytest
from fastapi.testclient import TestClient
from app.main import app
import zipfile
import io

client = TestClient(app)

def test_generate_facade_api():
    payload = {
        "building_geometry": '{"type": "Polygon", "coordinates": [[[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]]]}',
        "ground_alt_m": 10.0,
        "building_height_m": 50.0,
        "viewpoint_a": {"lat": 10.0, "lon": 10.0},
        "drone_model": "matrice_400",
        "camera_model": "riebo_dg6p_oblique"
    }

    response = client.post("/api/v1/planner/generate_facade_kmz", json=payload)

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/zip"

    # Read zip
    mem_zip = io.BytesIO(response.content)
    with zipfile.ZipFile(mem_zip, 'r') as zipf:
        files = zipf.namelist()
        assert "Face_A.kmz" in files
        assert "Face_B.kmz" in files
        assert "mission_summary.json" in files
