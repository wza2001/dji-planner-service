import os
import io
import zipfile
import requests

# 宿主机映射服务地址
BASE_URL = "http://localhost:8085"
INPUT_FILE = r"G:\我的云端硬盘\freedo\Private Work Folder\Projects\Luke\Test\test1.kml"
OUTPUT_KMZ = r"G:\我的云端硬盘\freedo\Private Work Folder\Projects\Luke\Test\Test_polygon_1_route.kmz"


def test_generate_kmz_from_file_pipeline():
    if not os.path.exists(INPUT_FILE):
        raise FileNotFoundError(f"找不到测试文件: {INPUT_FILE}")

    # 1. 切换为设备注册中心驱动的参数表，无需再手动指定 fov_h / fov_v
    form_data = {
        "drone_model": "matrice_400",
        "camera_model": "riebo_dg6p_oblique",
        "agl": 280.0,
        "speed": 14.0,
        "overlap_f": 0.8,
        "overlap_s": 0.7,
        "waypoint_mode": "sparse",
    }

    # 根据文件扩展名指定对应的 MIME 类型
    file_ext = os.path.splitext(INPUT_FILE)[1].lower()
    content_type = (
        "application/vnd.google-earth.kml+xml"
        if file_ext == ".kml"
        else "application/vnd.google-earth.kmz"
    )

    print(f"正在上传测区边界 [{os.path.basename(INPUT_FILE)}] 并请求规划航线...")
    print(f"  -> 目标机型: {form_data['drone_model']}, 挂载相机: {form_data['camera_model']}")

    with open(INPUT_FILE, "rb") as f:
        files = {
            "file": (os.path.basename(INPUT_FILE), f, content_type)
        }
        response = requests.post(
            f"{BASE_URL}/api/v1/planner/generate_kmz_from_file",
            data=form_data,
            files=files,
            timeout=60,
        )

    print(f"HTTP 状态码: {response.status_code}")
    if response.status_code != 200:
        print("服务返回错误:", response.text)
        return

    # 2. 响应类型断言
    assert "kmz" in response.headers.get("content-type", "").lower(), "返回 Content-Type 不是 KMZ"

    # 3. 内存解压并深入校验 WPML 规范
    kmz_bytes = io.BytesIO(response.content)
    with zipfile.ZipFile(kmz_bytes, "r") as zip_ref:
        file_list = zip_ref.namelist()
        print("KMZ 内部清单:", file_list)

        assert any("template.kml" in name for name in file_list), "缺失 template.kml"
        assert any("waylines.wpml" in name for name in file_list), "缺失 waylines.wpml"

        template_path = next(name for name in file_list if "template.kml" in name)
        waylines_path = next(name for name in file_list if "waylines.wpml" in name)

        template_xml = zip_ref.read(template_path).decode("utf-8")
        waylines_xml = zip_ref.read(waylines_path).decode("utf-8")

        # 硬件枚举值校验 (M400: 103, PSDK 载荷: 65535)
        assert "<wpml:droneEnumValue>103</wpml:droneEnumValue>" in (template_xml + waylines_xml), \
            "未正确识别并写入 Matrice 400 枚举值 (103)"
        assert "<wpml:payloadEnumValue>65535</wpml:payloadEnumValue>" in (template_xml + waylines_xml), \
            "未正确识别并写入睿珀 DG6P PSDK 载荷枚举值 (65535)"
        print("  -> 设备枚举校验通过: droneEnumValue=103, payloadEnumValue=65535")

        # 无云台刚性安装与航向逻辑校验
        assert "<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>" not in waylines_xml, \
            "异常: 睿珀 DG6P (has_gimbal=False) 不应生成 gimbalRotate 云台控制动作"
        assert "followWayline" in (template_xml + waylines_xml), \
            "异常: 无云台设备应自动设置 waypointHeadingMode 为 followWayline"
        print("  -> 硬件动作特性校验通过: 成功过滤 gimbalRotate，航向已锁定 followWayline")

    # 4. 保存生成的航线文件
    os.makedirs(os.path.dirname(OUTPUT_KMZ), exist_ok=True)
    with open(OUTPUT_KMZ, "wb") as f:
        f.write(response.content)

    print(f"\n[PASS] 端到端测试通过！KMZ 文件已输出至:\n{OUTPUT_KMZ} (文件大小: {len(response.content)} 字节)")


if __name__ == "__main__":
    test_generate_kmz_from_file_pipeline()