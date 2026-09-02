# scripts/calc_params.py
import math

def calculate_flight_parameters(agl_m: float,
                              fov_h_deg: float,
                              fov_v_deg: float,
                              forward_overlap: float = 0.75,
                              side_overlap: float = 0.75,
                              flight_speed_m_s: float = 8.0) -> dict:
    """
    计算无人机航测的核心航线参数
    :param agl_m: 相对飞行高度 (m)
    :param fov_h_deg: 镜头水平视角 FOV (度)
    :param fov_v_deg: 镜头垂直视角 FOV (度)
    :param forward_overlap: 航向重叠率 (0.75 即 75%)
    :param side_overlap: 侧向重叠率 (0.75 即 75%)
    :param flight_speed_m_s: 飞行速度 (m/s)
    :return: 包含各项推导参数的字典
    """
    # 1. 单张照片地面覆盖范围 (Footprint)
    ground_width = 2 * agl_m * math.tan(math.radians(fov_h_deg / 2.0))
    ground_height = 2 * agl_m * math.tan(math.radians(fov_v_deg / 2.0))

    # 2. 航线间距 (Line Spacing / 侧向) 与 拍照航距 (Photo Spacing / 航向)
    line_spacing = ground_width * (1.0 - side_overlap)
    photo_spacing = ground_height * (1.0 - forward_overlap)

    # 3. 拍照时间间隔 (秒)
    photo_interval = photo_spacing / flight_speed_m_s if flight_speed_m_s > 0 else 0

    results = {
        "agl_m": agl_m,
        "ground_width_m": ground_width,
        "ground_height_m": ground_height,
        "line_spacing_m": line_spacing,
        "photo_spacing_m": photo_spacing,
        "photo_interval_s": photo_interval,
        "flight_speed_m_s": flight_speed_m_s
    }

    print("\n" + "=" * 50)
    print(f"🚁 航测物理参数计算结果 (相对高度: {agl_m}m)")
    print("=" * 50)
    print(f"📷 镜头单帧视野 : {ground_width:.2f}m (宽/侧向) × {ground_height:.2f}m (高/航向)")
    print(f"↔️  航线平行间距 : {line_spacing:.2f} 米 (Side Spacing)")
    print(f"↕️  触发拍照航距 : {photo_spacing:.2f} 米 (Forward Spacing)")
    print(f"⏱️  定距拍照间隔 : {photo_interval:.2f} 秒/张 (速度: {flight_speed_m_s}m/s)")
    print("=" * 50 + "\n")

    return results

if __name__ == "__main__":
    # 测试代码 (以大疆航测机型常规参数为例)
    calculate_flight_parameters(agl_m=120, fov_h_deg=84, fov_v_deg=60, forward_overlap=0.80, side_overlap=0.70)
