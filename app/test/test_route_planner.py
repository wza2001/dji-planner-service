import zipfile
import io
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)


def test_generate_kmz_pipeline():
    payload = {
        "fov_h": 84.0,
        "fov_v": 60.0,
        "agl": 120.0,
        "speed": 10.0,
        "overlap_f": 0.8,
        "overlap_s": 0.7,
        "drone_enum": 95,
        "payload_enum": 52,
        "waypoint_mode": "sparse",
        "boundary_geojson": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [
                            [
                                [54.373204, 24.469145],
                                [54.374204, 24.469145],
                                [54.374204, 24.470145],
                                [54.373204, 24.470145],
                                [54.373204, 24.469145]
                            ]
                        ]
                    },
                    "properties": {"name": "Test_Zone"}
                }
            ]
        }
    }

    # 1. 直接触发接口调用
    response = client.post("/api/v1/planner/generate_kmz", json=payload)

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/vnd.google-earth.kmz"

    # 2. 内存解压校验大疆 WPML 规范规范（无需手动改 .zip）
    kmz_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(kmz_bytes, "r") as zip_ref:
        file_list = zip_ref.namelist()
        print("KMZ 内部结构:", file_list)

        # 断言大疆核心文件是否存在
        assert any("template.kml" in f for f in file_list)
        assert any("waylines.wpml" in f for f in file_list)

    # 3. 保存至本地以供实机验证
    with open("debug_flight.kmz", "wb") as f:
        f.write(response.content)
    print("调试文件已输出至 debug_flight.kmz")


if __name__ == "__main__":
    test_generate_kmz_pipeline()