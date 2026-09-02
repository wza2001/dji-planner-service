# utils/geo_geometry.py
import numpy as np
from shapely.geometry import Polygon, LineString, Point

def get_meter_degree_factors(lat: float) -> tuple[float, float]:
    """根据当前纬度计算 1 度对应的经纬度米数换算系数"""
    meters_per_deg_lat = 111000.0
    meters_per_deg_lon = 111000.0 * np.cos(np.radians(lat))
    return meters_per_deg_lat, meters_per_deg_lon

def create_octagonal_buffer(geom, radius_meters: float, center_lat: float) -> Polygon:
    """
    根据给定的米制半径，围绕几何体生成 8 个顶点的正八边形近似圆缓冲区算子
    """
    m_lat, m_lon = get_meter_degree_factors(center_lat)
    deg_radius_lat = radius_meters / m_lat
    deg_radius_lon = radius_meters / m_lon

    centroid = geom.centroid
    cx, cy = centroid.x, centroid.y

    # 均匀生成 8 个采样角度 (正八边形)
    angles = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    oct_coords = []
    for angle in angles:
        px = cx + deg_radius_lon * np.cos(angle)
        py = cy + deg_radius_lat * np.sin(angle)
        oct_coords.append((px, py))
    oct_coords.append(oct_coords[0])  # 闭合首尾点

    return Polygon(oct_coords)

def interpolate_line_points(line: LineString, spacing_m: float, lat: float) -> list[tuple[float, float]]:
    """
    【已废弃：不再用于 sparse 模式航点生成】

    该函数保留仅用于其他需要细粒度米制采样的场景，以及旧版 dense 模式
    （waypoint_mode="dense"）的兼容。在 sparse（默认）模式下，
    route_planner.generate_grid_flight_path 已不再调用本函数，
    改为只保留扫描线端点与避障拐点，大幅压缩航点数量。

    在 LineString 线上按指定米制距离（spacing_m）等距采样补点。
    """
    m_lat, m_lon = get_meter_degree_factors(lat)
    coords = list(line.coords)
    if len(coords) < 2:
        return coords

    sampled_points = []
    for i in range(len(coords) - 1):
        p1, p2 = coords[i], coords[i+1]
        dx = (p2[0] - p1[0]) * m_lon
        dy = (p2[1] - p1[1]) * m_lat
        seg_length = np.sqrt(dx**2 + dy**2)

        sampled_points.append((p1[0], p1[1]))
        if seg_length > spacing_m:
            num_samples = int(np.floor(seg_length / spacing_m))
            for k in range(1, num_samples + 1):
                fraction = (k * spacing_m) / seg_length
                ix = p1[0] + (p2[0] - p1[0]) * fraction
                iy = p1[1] + (p2[1] - p1[1]) * fraction
                sampled_points.append((ix, iy))

    sampled_points.append((coords[-1][0], coords[-1][1]))
    return sampled_points
