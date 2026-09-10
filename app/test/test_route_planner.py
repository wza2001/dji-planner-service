import os
import io
import zipfile
import requests

# 目标指向宿主机映射的容器端口
BASE_URL = "http://localhost:8085"
INPUT_KMZ = r"G:\我的云端硬盘\freedo\Private Work Folder\Projects\Luke\Test\test1.kml"
OUTPUT_KMZ = r"G:\我的云端硬盘\freedo\Private Work Folder\Projects\Luke\Test\Test_polygon_1_route.kmz"


def test_generate_kmz_pipeline():
    if not os.path.exists(INPUT_KMZ):
        raise FileNotFoundError(f"找不到测试文件: {INPUT_KMZ}")

    data = {
        "fov_h": 84.0,
        "fov_v": 60.0,
        "agl": 120.0,
        "speed": 10.0,
        "overlap_f": 0.8,
        "overlap_s": 0.7,
        "drone_enum": 95,
        "payload_enum": 52,
        "waypoint_mode": "sparse",
    }

    print(f"正在向容器服务 {BASE_URL} 上传文件测试...")

    with open(INPUT_KMZ, "rb") as f:
        files = {
            "file": (os.path.basename(INPUT_KMZ), f, "application/vnd.google-earth.kmz")
        }
        response = requests.post(
            f"{BASE_URL}/api/v1/planner/generate_kmz_from_file",
            data=data,
            files=files,
            timeout=60,
        )

    print(f"HTTP 状态码: {response.status_code}")

    if response.status_code != 200:
        print("错误响应内容:", response.text)
        return

    # 1. 验证返回类型
    assert "kmz" in response.headers.get("content-type", "").lower(), "返回 Content-Type 不是 KMZ"

    # 2. 内存解压校验大疆 WPML 规范
    kmz_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(kmz_bytes, "r") as zip_ref:
        file_list = zip_ref.namelist()
        print("KMZ 内部清单:", file_list)
        assert any("template.kml" in name for name in file_list), "缺失 template.kml"
        assert any("waylines.wpml" in name for name in file_list), "缺失 waylines.wpml"

    # 3. 写入输出文件
    os.makedirs(os.path.dirname(OUTPUT_KMZ), exist_ok=True)
    with open(OUTPUT_KMZ, "wb") as f:
        f.write(response.content)

    print(f"测试通过！航线 KMZ 已保存至: {OUTPUT_KMZ} (大小: {len(response.content)} 字节)")


if __name__ == "__main__":
    test_generate_kmz_pipeline()