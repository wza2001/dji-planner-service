import pytest
from app.models.schemas import FacadeRequest, Viewpoint
from app.services.facade_service import generate_facade_kmz_bundle
import os
import zipfile

def test_generate_facade_kmz():
    request = FacadeRequest(
        building_geometry='{"type": "Polygon", "coordinates": [[[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0], [0.0, 0.0]]]}',
        ground_alt_m=10.0,
        building_height_m=50.0,
        viewpoint_a=Viewpoint(lat=10.0, lon=10.0),
        drone_model="matrice_400",
        camera_model="riebo_dg6p_oblique"
    )

    zip_path = generate_facade_kmz_bundle(request)

    assert os.path.exists(zip_path)

    with zipfile.ZipFile(zip_path, 'r') as zipf:
        files = zipf.namelist()
        assert "Face_A.kmz" in files
        assert "Face_B.kmz" in files
        assert "mission_summary.json" in files

    os.remove(zip_path)
