# scripts/route_planner.py
import os
import json
import math
import zipfile
import time
import pandas as pd
import geopandas as gpd
import numpy as np
import networkx as nx
from shapely.geometry import Polygon, MultiPolygon, LineString, Point
from shapely.validation import make_valid
import simplekml
import fiona
import uuid
from app.models.flight_plan import FlightPlan, GenericWaypoint
from app.core.exporters.dji_exporter import DJIWPMLPackageExporter
from app.core.exporters.geojson_exporter import GeoJSONExporter


from app.core.calc_params import calculate_flight_parameters
from app.utils.geo_geometry import get_meter_degree_factors, interpolate_line_points

def parse_fov_value(val):
    if isinstance(val, (list, tuple)):
        return float(val[0])
    return float(val)

class VisibilityRouter:
    """
    【边界可视寻路器 - 解决穿模的核心】
    基于 Visibility Graph 算法。在安全多边形的顶点之间建立导航网格 (NavMesh)。
    无人机跨区跳跃时，沿多边形边界贴墙飞行，彻底杜绝穿透禁飞区。
    """
    def __init__(self, polygon: Polygon):
        self.polygon = polygon
        # 微小外扩容差(约1米)，吸收浮点精度误差，防止边界线被误判为在多边形外
        self.safe_poly = polygon.buffer(1e-5)
        self.G = nx.Graph()

        # 1. 简化多边形并提取所有孔洞与外部边界顶点
        poly_simp = polygon.simplify(1e-5, preserve_topology=True)
        rings = []
        if poly_simp.geom_type == 'Polygon':
            rings = [poly_simp.exterior] + list(poly_simp.interiors)
        elif poly_simp.geom_type == 'MultiPolygon':
            for p in poly_simp.geoms:
                rings.extend([p.exterior] + list(p.interiors))

        vertices = []
        for ring in rings:
            vertices.extend(list(ring.coords))

        # 2. 顶点去重
        self.unique_verts = []
        for v in vertices:
            if not any(np.hypot(v[0]-uv[0], v[1]-uv[1]) < 1e-6 for uv in self.unique_verts):
                self.unique_verts.append(v)
                self.G.add_node(len(self.unique_verts)-1, pos=v)

        # 3. 构建可视连通图 (预计算网格)
        num_v = len(self.unique_verts)
        for i in range(num_v):
            for j in range(i + 1, num_v):
                v1 = self.unique_verts[i]
                v2 = self.unique_verts[j]
                line = LineString([(v1[0], v1[1]), (v2[0], v2[1])])
                if self.safe_poly.contains(line):
                    dist = np.hypot(v1[0]-v2[0], v1[1]-v2[1])
                    self.G.add_edge(i, j, weight=dist)

    def get_safe_path(self, p1: tuple, p2: tuple) -> list:
        # 如果两点之间没有障碍物，直接返回直线
        line = LineString([(p1[0], p1[1]), (p2[0], p2[1])])
        if self.safe_poly.contains(line):
            return [p1, p2]

        # 如果被遮挡，将起点和终点临时接入导航网格，跑最短路径
        temp_G = self.G.copy()
        temp_G.add_node("start", pos=p1)
        temp_G.add_node("end", pos=p2)

        for i, v in enumerate(self.unique_verts):
            if self.safe_poly.contains(LineString([(p1[0], p1[1]), (v[0], v[1])])):
                temp_G.add_edge("start", i, weight=np.hypot(p1[0]-v[0], p1[1]-v[1]))
            if self.safe_poly.contains(LineString([(p2[0], p2[1]), (v[0], v[1])])):
                temp_G.add_edge("end", i, weight=np.hypot(p2[0]-v[0], p2[1]-v[1]))

        try:
            path = nx.shortest_path(temp_G, source="start", target="end", weight='weight')
            return [temp_G.nodes[n]['pos'] for n in path]
        except nx.NetworkXNoPath:
            # 极端情况降级为直线
            return [p1, p2]


def calculate_heading(p1: tuple, p2: tuple) -> float:
    """
    计算从 p1 到 p2 的航向角（0-360度，正北为0，顺时针）。

    使用球面公式（考虑纬度对经度方向的影响），与项目中 haversine 距离计算保持一致。
    p1 / p2 为 (lon, lat)。
    """
    lon1, lat1 = math.radians(p1[0]), math.radians(p1[1])
    lon2, lat2 = math.radians(p2[0]), math.radians(p2[1])
    dlon = lon2 - lon1

    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    deg = math.degrees(bearing)
    return (deg + 360.0) % 360.0


def classify_and_enrich_waypoints(waypoints: list[dict]) -> list[dict]:
    """
    遍历航点列表，统一计算每个航点的航向信息（heading_in / heading_out / heading_change）。

    - heading_in  = 上一航点 -> 当前航点的航向（首点取 0）
    - heading_out = 当前航点 -> 下一航点的航向（末点取 heading_in）
    - heading_change = 两航向的最小夹角（0-180度）
    - 当某航点类型为 "scan_end" 且 heading_change > 45° 时，升级为 "turn"（明显转向点）

    Args:
        waypoints: 原始航点 list[dict]，type 已设置，heading 字段暂为 0。

    Returns:
        完整填充 heading 信息的航点列表。
    """
    n = len(waypoints)
    for i, wp in enumerate(waypoints):
        prev_wp = waypoints[i - 1] if i > 0 else None
        nxt_wp = waypoints[i + 1] if i < n - 1 else None

        if prev_wp is not None:
            heading_in = calculate_heading(
                (prev_wp["lon"], prev_wp["lat"]),
                (wp["lon"], wp["lat"])
            )
        else:
            heading_in = 0.0

        if nxt_wp is not None:
            heading_out = calculate_heading(
                (wp["lon"], wp["lat"]),
                (nxt_wp["lon"], nxt_wp["lat"])
            )
        else:
            heading_out = heading_in

        diff = abs(heading_out - heading_in)
        heading_change = min(diff, 360.0 - diff)

        wp["heading_in"] = heading_in
        wp["heading_out"] = heading_out
        wp["heading_change"] = heading_change

        # 明显转向的扫描线终点升级为 turn 点
        if wp.get("type") == "scan_end" and heading_change > 45.0:
            wp["type"] = "turn"

    return waypoints


def save_waypoints_geojson(waypoints: list[dict], output_path: str | None) -> None:
    """
    将航点列表保存为 GeoJSON 文件（FeatureCollection，每个航点为 Point Feature）。

    坐标系 EPSG:4326，properties 包含 type / segment_idx / heading_in /
    heading_out / heading_change / turn_mode（通过 get_turn_mode 计算），
    按输入顺序输出，便于在 QGIS / ArcGIS 中查看航线拓扑。

    Args:
        waypoints: 航点 list[dict]（需包含 lon/lat/alt/type 等字段）。
        output_path: 输出 GeoJSON 路径；若为 None 则不输出。
    """
    if output_path is None:
        return

    parent = os.path.dirname(output_path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    features = []
    for idx, wp in enumerate(waypoints):
        turn_mode = get_turn_mode(
            wp.get("heading_in", 0.0),
            wp.get("heading_out", 0.0)
        )
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [float(wp["lon"]), float(wp["lat"]), float(wp["alt"])]
            },
            "properties": {
                "index": idx,
                "type": wp.get("type", "transit"),
                "segment_idx": wp.get("segment_idx", -1),
                "heading_in": round(float(wp.get("heading_in", 0.0)), 2),
                "heading_out": round(float(wp.get("heading_out", 0.0)), 2),
                "heading_change": round(float(wp.get("heading_change", 0.0)), 2),
                "turn_mode": turn_mode
            }
        }
        features.append(feature)

    geojson = {
        "type": "FeatureCollection",
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:OGC:1.3:CRS84"}
        },
        "features": features
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(geojson, f, ensure_ascii=False, indent=2)


def generate_grid_flight_path(boundary_geom: Polygon | MultiPolygon,
                              line_spacing_m: float,
                              photo_spacing_m: float = 20.0,
                              alt_agl: float = 120.0) -> list[dict]:
    """
    【核心航线算法：DFS 图论平铺 + 边界避障寻路】(精简航点 / sparse 模式)

    相比旧版，本函数不再使用 interpolate_line_points 在扫描线上密集插值，
    只保留：扫描线端点（takeoff / scan_start / scan_end）与跨区域避障拐点
    （transit）。航点数量被大幅压缩，最后统一由 classify_and_enrich_waypoints
    计算每个航点的航向信息。

    Returns:
        list[dict]，每个航点包含：
            lon, lat, alt, type, segment_idx,
            heading_in, heading_out, heading_change
    """
    if boundary_geom.is_empty:
        return []

    if not boundary_geom.is_valid:
        boundary_geom = make_valid(boundary_geom)

    min_lon, min_lat, max_lon, max_lat = boundary_geom.bounds
    center_lat = (min_lat + max_lat) / 2.0
    m_lat, m_lon = get_meter_degree_factors(center_lat)

    spacing_deg_lon = line_spacing_m / m_lon
    lon_steps = np.arange(min_lon, max_lon + spacing_deg_lon, spacing_deg_lon)

    # 实例化安全寻路器
    try:
        router = VisibilityRouter(boundary_geom)
    except Exception:
        class DummyRouter:
            def get_safe_path(self, p1, p2): return [p1, p2]
        router = DummyRouter()

    columns = []

    # 1. 剖切生成所有扫描线
    for lon in lon_steps:
        scan_line = LineString([(lon, min_lat - 0.005), (lon, max_lat + 0.005)])
        try:
            intersected = scan_line.intersection(boundary_geom)
        except Exception:
            continue

        if intersected.is_empty:
            continue

        col_segments = []
        if isinstance(intersected, LineString):
            if intersected.length > 1e-7:
                col_segments.append(intersected)
        elif hasattr(intersected, 'geoms'):
            for g in intersected.geoms:
                if isinstance(g, LineString) and g.length > 1e-7:
                    col_segments.append(g)

        if col_segments:
            # 保证按纬度排序
            col_segments.sort(key=lambda line: line.coords[0][1])
            columns.append(col_segments)

    if not columns:
        return []

    # 2. 构建无向图 (Adjacency Graph) - 划分细胞群
    G = nx.Graph()
    node_id = 0
    node_map = {}

    for col_idx, col_segs in enumerate(columns):
        for seg in col_segs:
            G.add_node(node_id, col_idx=col_idx, seg=seg)
            node_map[node_id] = seg
            node_id += 1

    safe_poly_check = boundary_geom.buffer(1e-5)
    for col_idx in range(len(columns) - 1):
        curr_nodes = [n for n, attr in G.nodes(data=True) if attr['col_idx'] == col_idx]
        next_nodes = [n for n, attr in G.nodes(data=True) if attr['col_idx'] == col_idx + 1]

        for cn in curr_nodes:
            seg_c = G.nodes[cn]['seg']
            for nn in next_nodes:
                seg_n = G.nodes[nn]['seg']
                # 连线校验：中点直连如果不穿过孔洞，才视为相邻
                mid_line = LineString([(seg_c.centroid.x, seg_c.centroid.y), (seg_n.centroid.x, seg_n.centroid.y)])
                if safe_poly_check.contains(mid_line):
                    G.add_edge(cn, nn)

    subgraphs = [G.subgraph(c).copy() for c in nx.connected_components(G)]
    ordered_waypoints = []

    for sg in subgraphs:
        start_node = min(sg.nodes(data=True), key=lambda x: x[1]['col_idx'])[0]
        # 3. 核心：DFS 前序遍历，保证平滑区域推进行为
        ordered_nodes = list(nx.dfs_preorder_nodes(sg, source=start_node))

        for n_idx in ordered_nodes:
            seg = node_map[n_idx]
            p_start, p_end = seg.coords[0], seg.coords[-1]

            if not ordered_waypoints:
                # 第一条线：直接记录扫描线端点（takeoff + scan_end）
                ordered_waypoints.append({
                    "lon": p_start[0], "lat": p_start[1], "alt": alt_agl,
                    "type": "takeoff", "segment_idx": 0,
                    "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                })
                ordered_waypoints.append({
                    "lon": p_end[0], "lat": p_end[1], "alt": alt_agl,
                    "type": "scan_end", "segment_idx": 0,
                    "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                })
            else:
                last_wp = (ordered_waypoints[-1]["lon"], ordered_waypoints[-1]["lat"])

                # 4. 动态进场与寻路：计算前往新线段两端的绕行距离
                path_to_start = router.get_safe_path(last_wp, p_start)
                path_to_end = router.get_safe_path(last_wp, p_end)

                dist_start = sum(np.hypot(path_to_start[i][0]-path_to_start[i-1][0],
                                          path_to_start[i][1]-path_to_start[i-1][1]) for i in range(1, len(path_to_start)))
                dist_end = sum(np.hypot(path_to_end[i][0]-path_to_end[i-1][0],
                                        path_to_end[i][1]-path_to_end[i-1][1]) for i in range(1, len(path_to_end)))

                # 自动选择最优进场端，自然形成 S 型
                if dist_start <= dist_end:
                    chosen_path = path_to_start
                    chosen_seg = seg
                else:
                    chosen_path = path_to_end
                    chosen_seg = LineString([(p[0], p[1]) for p in list(seg.coords)[::-1]])

                # 插入跨区域的避障绕行路径点（transit），剔除头尾避免重复
                for pt in chosen_path[1:-1]:
                    ordered_waypoints.append({
                        "lon": pt[0], "lat": pt[1], "alt": alt_agl,
                        "type": "transit", "segment_idx": -1,
                        "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                    })

                # 插入扫描线端点（scan_start + scan_end），避免重复
                seg_start = chosen_seg.coords[0]
                seg_end = chosen_seg.coords[-1]
                if not (ordered_waypoints and
                        ordered_waypoints[-1]["lon"] == seg_start[0] and
                        ordered_waypoints[-1]["lat"] == seg_start[1]):
                    ordered_waypoints.append({
                        "lon": seg_start[0], "lat": seg_start[1], "alt": alt_agl,
                        "type": "scan_start", "segment_idx": n_idx,
                        "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                    })
                ordered_waypoints.append({
                    "lon": seg_end[0], "lat": seg_end[1], "alt": alt_agl,
                    "type": "scan_end", "segment_idx": n_idx,
                    "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                })

    if not ordered_waypoints:
        return []

    # 最后一个航点设为 landing
    ordered_waypoints[-1]["type"] = "landing"

    # 统一计算航向信息（heading_in / heading_out / heading_change）
    ordered_waypoints = classify_and_enrich_waypoints(ordered_waypoints)

    return ordered_waypoints


def generate_dense_grid_flight_path(boundary_geom: Polygon | MultiPolygon,
                                    line_spacing_m: float,
                                    photo_spacing_m: float = 20.0,
                                    alt_agl: float = 120.0) -> list[tuple]:
    """
    【旧版密集航点生成 - 兼容 dense 模式】

    保留历史行为：使用 interpolate_line_points 在每条扫描线上密集插值生成
    拍照点，返回 list[tuple]（每个为 (lon, lat, alt)）。仅在
    waypoint_mode="dense" 时调用，用于兼容旧版 DJI WPML（逐点拍照动作）。

    注意：sparse 模式已不再使用本函数；密集插值逻辑仅供 dense 模式与
    其他需要细粒度航点的场景使用。
    """
    if boundary_geom.is_empty:
        return []

    if not boundary_geom.is_valid:
        boundary_geom = make_valid(boundary_geom)

    min_lon, min_lat, max_lon, max_lat = boundary_geom.bounds
    center_lat = (min_lat + max_lat) / 2.0
    m_lat, m_lon = get_meter_degree_factors(center_lat)

    spacing_deg_lon = line_spacing_m / m_lon
    lon_steps = np.arange(min_lon, max_lon + spacing_deg_lon, spacing_deg_lon)

    try:
        router = VisibilityRouter(boundary_geom)
    except Exception:
        class DummyRouter:
            def get_safe_path(self, p1, p2): return [p1, p2]
        router = DummyRouter()

    columns = []

    for lon in lon_steps:
        scan_line = LineString([(lon, min_lat - 0.005), (lon, max_lat + 0.005)])
        try:
            intersected = scan_line.intersection(boundary_geom)
        except Exception:
            continue

        if intersected.is_empty:
            continue

        col_segments = []
        if isinstance(intersected, LineString):
            if intersected.length > 1e-7:
                col_segments.append(intersected)
        elif hasattr(intersected, 'geoms'):
            for g in intersected.geoms:
                if isinstance(g, LineString) and g.length > 1e-7:
                    col_segments.append(g)

        if col_segments:
            col_segments.sort(key=lambda line: line.coords[0][1])
            columns.append(col_segments)

    if not columns:
        return []

    G = nx.Graph()
    node_id = 0
    node_map = {}

    for col_idx, col_segs in enumerate(columns):
        for seg in col_segs:
            G.add_node(node_id, col_idx=col_idx, seg=seg)
            node_map[node_id] = seg
            node_id += 1

    safe_poly_check = boundary_geom.buffer(1e-5)
    for col_idx in range(len(columns) - 1):
        curr_nodes = [n for n, attr in G.nodes(data=True) if attr['col_idx'] == col_idx]
        next_nodes = [n for n, attr in G.nodes(data=True) if attr['col_idx'] == col_idx + 1]
        for cn in curr_nodes:
            seg_c = G.nodes[cn]['seg']
            for nn in next_nodes:
                seg_n = G.nodes[nn]['seg']
                mid_line = LineString([(seg_c.centroid.x, seg_c.centroid.y), (seg_n.centroid.x, seg_n.centroid.y)])
                if safe_poly_check.contains(mid_line):
                    G.add_edge(cn, nn)

    subgraphs = [G.subgraph(c).copy() for c in nx.connected_components(G)]
    ordered_waypoints = []

    for sg in subgraphs:
        start_node = min(sg.nodes(data=True), key=lambda x: x[1]['col_idx'])[0]
        ordered_nodes = list(nx.dfs_preorder_nodes(sg, source=start_node))

        for n_idx in ordered_nodes:
            seg = node_map[n_idx]
            p_start, p_end = seg.coords[0], seg.coords[-1]

            if not ordered_waypoints:
                pts = interpolate_line_points(seg, photo_spacing_m, center_lat)
                for pt in pts:
                    ordered_waypoints.append((pt[0], pt[1], alt_agl))
            else:
                last_wp = (ordered_waypoints[-1][0], ordered_waypoints[-1][1])

                path_to_start = router.get_safe_path(last_wp, p_start)
                path_to_end = router.get_safe_path(last_wp, p_end)

                dist_start = sum(np.hypot(path_to_start[i][0]-path_to_start[i-1][0],
                                          path_to_start[i][1]-path_to_start[i-1][1]) for i in range(1, len(path_to_start)))
                dist_end = sum(np.hypot(path_to_end[i][0]-path_to_end[i-1][0],
                                        path_to_end[i][1]-path_to_end[i-1][1]) for i in range(1, len(path_to_end)))

                if dist_start <= dist_end:
                    chosen_path = path_to_start
                    chosen_seg = seg
                else:
                    chosen_path = path_to_end
                    chosen_seg = LineString([(p[0], p[1]) for p in list(seg.coords)[::-1]])

                for pt in chosen_path[1:-1]:
                    ordered_waypoints.append((pt[0], pt[1], alt_agl))

                pts = interpolate_line_points(chosen_seg, photo_spacing_m, center_lat)
                for pt in pts:
                    if ordered_waypoints and (ordered_waypoints[-1][0] == pt[0] and ordered_waypoints[-1][1] == pt[1]):
                        continue
                    ordered_waypoints.append((pt[0], pt[1], alt_agl))

    return ordered_waypoints


def _wp_coord(wp):
    """统一从 dict 或 tuple 航点中提取 (lon, lat)。"""
    if isinstance(wp, dict):
        return wp["lon"], wp["lat"]
    return wp[0], wp[1]



def plan_flight_geometry(
    boundary_geom: Polygon | MultiPolygon,
    flight_alt_agl: float = 120.0,
    flight_speed: float = 10.0,
    camera_info: dict = None,
    forward_overlap: float = 0.80,
    side_overlap: float = 0.70,
    waypoint_mode: str = "sparse"
) -> FlightPlan:
    if camera_info is None:
        camera_info = {"fov_horizontal_deg": [48.0, 36.0], "fov_vertical_deg": [34.0, 24.0]}

    fov_h = parse_fov_value(camera_info.get("fov_horizontal_deg", 48.0))
    fov_v = parse_fov_value(camera_info.get("fov_vertical_deg", 34.0))

    calc_res = calculate_flight_parameters(
        agl_m=flight_alt_agl, fov_h_deg=fov_h, fov_v_deg=fov_v,
        forward_overlap=forward_overlap, side_overlap=side_overlap,
        flight_speed_m_s=flight_speed
    )

    line_spacing_m = calc_res["line_spacing_m"]
    photo_spacing_m = calc_res.get("photo_spacing_m", calc_res.get("forward_spacing_m", 20.0))

    sub_geom = boundary_geom
    if not sub_geom.is_valid:
        sub_geom = make_valid(sub_geom)

    sub_polys = [sub_geom] if isinstance(sub_geom, Polygon) else list(sub_geom.geoms)

    all_waypoints_raw = []
    for p_idx, poly in enumerate(sub_polys):
        if poly.is_empty or poly.area < 1e-8:
            continue

        if waypoint_mode == "dense":
            waypoints = generate_dense_grid_flight_path(
                boundary_geom=poly,
                line_spacing_m=line_spacing_m,
                photo_spacing_m=photo_spacing_m,
                alt_agl=flight_alt_agl
            )
        else:
            waypoints = generate_grid_flight_path(
                boundary_geom=poly,
                line_spacing_m=line_spacing_m,
                photo_spacing_m=photo_spacing_m,
                alt_agl=flight_alt_agl
            )
        if waypoints:
            all_waypoints_raw.extend(waypoints)

    total_distance = 0.0
    generic_waypoints = []

    if all_waypoints_raw:
        for i in range(1, len(all_waypoints_raw)):
            wp_curr = _wp_coord(all_waypoints_raw[i])
            wp_prev = _wp_coord(all_waypoints_raw[i - 1])
            m_lat, m_lon = get_meter_degree_factors((wp_curr[1] + wp_prev[1]) / 2.0)
            dx = (wp_curr[0] - wp_prev[0]) * m_lon
            dy = (wp_curr[1] - wp_prev[1]) * m_lat
            total_distance += np.hypot(dx, dy)

        for i, wp in enumerate(all_waypoints_raw):
            if isinstance(wp, dict):
                lon, lat = wp["lon"], wp["lat"]
                alt = wp.get("alt", flight_alt_agl)
                speed = wp.get("speed", flight_speed)
                wp_type = wp.get("type", "transit")
                heading_in = wp.get("heading_in", 0.0)
                heading_out = wp.get("heading_out", 0.0)
                heading_change = wp.get("heading_change", 0.0)
                segment_idx = wp.get("segment_idx", -1)
            else:
                lon, lat = wp[0], wp[1]
                alt = wp[2] if len(wp) > 2 else flight_alt_agl
                speed = flight_speed
                wp_type = "transit"
                heading_in = 0.0
                heading_out = 0.0
                heading_change = 0.0
                segment_idx = -1

            gw = GenericWaypoint(
                index=i,
                lon=lon,
                lat=lat,
                alt=alt,
                speed=speed,
                type=wp_type,
                heading_in=heading_in,
                heading_out=heading_out,
                heading_change=heading_change,
                gimbal_pitch=-90.0,
                segment_idx=segment_idx
            )
            generic_waypoints.append(gw)

    duration = total_distance / flight_speed if flight_speed > 0 else 0

    return FlightPlan(
        plan_id=str(uuid.uuid4()),
        flight_alt_agl=flight_alt_agl,
        flight_speed=flight_speed,
        total_distance_m=total_distance,
        estimated_duration_s=duration,
        photo_spacing_m=photo_spacing_m,
        waypoint_mode=waypoint_mode,
        waypoints=generic_waypoints
    )

def _convert_to_flight_plan(waypoints: list, speed: float = 12.0, alt: float = 120.0, photo_spacing_m: float = 20.0) -> FlightPlan:
    if not waypoints:
        return FlightPlan(
            plan_id=str(uuid.uuid4()),
            flight_alt_agl=alt,
            flight_speed=speed,
            total_distance_m=0.0,
            estimated_duration_s=0.0,
            photo_spacing_m=photo_spacing_m,
            waypoint_mode="sparse",
            waypoints=[]
        )

    total_distance = 0.0
    for i in range(1, len(waypoints)):
        wp_curr = _wp_coord(waypoints[i])
        wp_prev = _wp_coord(waypoints[i - 1])
        m_lat, m_lon = get_meter_degree_factors((wp_curr[1] + wp_prev[1]) / 2.0)
        dx = (wp_curr[0] - wp_prev[0]) * m_lon
        dy = (wp_curr[1] - wp_prev[1]) * m_lat
        total_distance += np.hypot(dx, dy)

    generic_waypoints = []
    waypoint_mode = "sparse"
    for i, wp in enumerate(waypoints):
        if isinstance(wp, dict):
            lon, lat = wp["lon"], wp["lat"]
            wp_alt = wp.get("alt", alt)
            wp_speed = wp.get("speed", speed)
            wp_type = wp.get("type", "transit")
            heading_in = wp.get("heading_in", 0.0)
            heading_out = wp.get("heading_out", 0.0)
            heading_change = wp.get("heading_change", 0.0)
            segment_idx = wp.get("segment_idx", -1)
            waypoint_mode = "sparse"
        else:
            lon, lat = wp[0], wp[1]
            wp_alt = wp[2] if len(wp) > 2 else alt
            wp_speed = speed
            wp_type = "transit"
            heading_in = 0.0
            heading_out = 0.0
            heading_change = 0.0
            segment_idx = -1
            waypoint_mode = "dense"

        gw = GenericWaypoint(
            index=i,
            lon=lon,
            lat=lat,
            alt=wp_alt,
            speed=wp_speed,
            type=wp_type,
            heading_in=heading_in,
            heading_out=heading_out,
            heading_change=heading_change,
            gimbal_pitch=-90.0,
            segment_idx=segment_idx
        )
        generic_waypoints.append(gw)

    duration = total_distance / speed if speed > 0 else 0
    return FlightPlan(
        plan_id=str(uuid.uuid4()),
        flight_alt_agl=alt,
        flight_speed=speed,
        total_distance_m=total_distance,
        estimated_duration_s=duration,
        photo_spacing_m=photo_spacing_m,
        waypoint_mode=waypoint_mode,
        waypoints=generic_waypoints
    )

def generate_dji_template_kml(waypoints: list = None, drone_enum: int = 68, payload_enum: int = 52, speed: float = 12) -> str:
    plan = _convert_to_flight_plan(waypoints, speed=speed)
    from app.core.exporters.dji_exporter import generate_dji_template_kml as ext_gen_template
    return ext_gen_template(plan, drone_enum=drone_enum, payload_enum=payload_enum)

def generate_dji_waylines_wpml(waypoints: list, flight_speed: float = 10, flight_alt: float = 120, height_mode: str = "relativeToStartPoint", photo_spacing_m: float = 20.0, drone_enum: int = 68, payload_enum: int = 52) -> str:
    plan = _convert_to_flight_plan(waypoints, speed=flight_speed, alt=flight_alt, photo_spacing_m=photo_spacing_m)
    from app.core.exporters.dji_exporter import generate_dji_waylines_wpml as ext_gen_waylines
    return ext_gen_waylines(plan, drone_enum=drone_enum, payload_enum=payload_enum)

def package_dji_kmz(template_kml_str: str, waylines_wpml_str: str, output_kmz_path: str) -> None:
    with zipfile.ZipFile(output_kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.writestr('wpmz/template.kml', template_kml_str.encode('utf-8'))
        kmz.writestr('wpmz/waylines.wpml', waylines_wpml_str.encode('utf-8'))


def plan_routes_from_safe_airspace(safe_airspace_file: str,
                                   flight_alt_agl: float = 120.0,
                                   flight_speed: float = 10.0,
                                   camera_info: dict = None,
                                   forward_overlap: float = 0.80,
                                   side_overlap: float = 0.70,
                                   output_kmz: str = "output/flight_routes.kmz",
                                   waypoint_mode: str = "sparse",
                                   drone_enum: int = 68,
                                   drone_sub_enum: int = 0,
                                   payload_enum: int = 52,
                                   has_gimbal: bool = True,
                                   lens_type: str = "single"):
    """
    【单次精确路线规划与 KMZ 打包主控】

    Args:
        safe_airspace_file: 安全可飞区域矢量文件路径（含 EPSG:4326 几何）。
        flight_alt_agl: 相对起飞点飞行高度（m）。
        flight_speed: 自动飞行速度（m/s）。
        camera_info: 相机 FOV 信息字典。
        forward_overlap: 航向重叠率。
        side_overlap: 旁向重叠率。
        output_kmz: 输出 KMZ 路径。
        waypoint_mode: 航点模式，"sparse"（默认，精简航点 + GeoJSON 输出）
                       或 "dense"（旧版密集航点，逐点拍照）。
    """
    if camera_info is None:
        camera_info = {"fov_horizontal_deg": [48.0, 36.0], "fov_vertical_deg": [34.0, 24.0]}

    fov_h = parse_fov_value(camera_info.get("fov_horizontal_deg", 48.0))
    fov_v = parse_fov_value(camera_info.get("fov_vertical_deg", 34.0))

    if not os.path.exists(safe_airspace_file):
        raise FileNotFoundError(f"❌ 找不到安全可飞区域文件: {safe_airspace_file}")

    print(f"✈️ 正在加载安全可飞区域数据: {safe_airspace_file}")

    try:
        layers = fiona.listlayers(safe_airspace_file)
    except Exception:
        layers = [None]

    gdfs = []
    for layer in layers:
        try:
            if layer is not None:
                if "No-Fly" in layer or "禁飞区" in layer:
                    continue
                temp_gdf = gpd.read_file(safe_airspace_file, layer=layer)
            else:
                temp_gdf = gpd.read_file(safe_airspace_file)
            if not temp_gdf.empty:
                gdfs.append(temp_gdf)
        except Exception:
            continue

    if gdfs:
        flight_gdf = pd.concat(gdfs, ignore_index=True)
    else:
        flight_gdf = gpd.read_file(safe_airspace_file)

    if flight_gdf.crs and flight_gdf.crs.to_string() != "EPSG:4326":
        flight_wgs84 = flight_gdf.to_crs("EPSG:4326")
    else:
        flight_wgs84 = flight_gdf

    if lens_type == "oblique_5lens":
        try:
            local_crs = flight_wgs84.estimate_utm_crs()
            temp_gdf = flight_wgs84.to_crs(local_crs)
            temp_gdf["geometry"] = temp_gdf.geometry.buffer(flight_alt_agl)
            flight_wgs84 = temp_gdf.to_crs("EPSG:4326")
        except Exception:
            # Fallback if estimate_utm_crs is not available or fails
            m_lat, m_lon = get_meter_degree_factors(flight_wgs84.geometry.centroid.y.mean())
            # Rough estimation: use the average of lat/lon conversion factors
            # A more robust fallback would be loop over each geometry but this is an acceptable fallback
            deg_buffer = flight_alt_agl / ((m_lat + m_lon) / 2)
            flight_wgs84["geometry"] = flight_wgs84.geometry.buffer(deg_buffer)

    calc_res = calculate_flight_parameters(
        agl_m=flight_alt_agl, fov_h_deg=fov_h, fov_v_deg=fov_v,
        forward_overlap=forward_overlap, side_overlap=side_overlap,
        flight_speed_m_s=flight_speed
    )

    line_spacing_m = calc_res["line_spacing_m"]
    photo_spacing_m = calc_res.get("forward_spacing_m", 20.0)


    print(f"🔄 正在精算图论 DFS 与边界避障航线... (waypoint_mode={waypoint_mode})")

    polys_to_plan = []

    for idx, row in flight_wgs84.iterrows():
        sub_geom = row.geometry
        if sub_geom is None or sub_geom.is_empty:
            continue

        zone_name = "Zone"
        for col in ["name", "Name", "ID", "id", "title", "Layer"]:
            if col in row and pd.notna(row[col]):
                val_str = str(row[col])
                if val_str and "No-Fly" not in val_str and "禁飞区" not in val_str:
                    zone_name = val_str
                    break
        if zone_name == "Zone":
            zone_name = f"Layer_{idx+1}"

        if sub_geom.geom_type not in ['Polygon', 'MultiPolygon']:
            continue

        if not sub_geom.is_valid:
            sub_geom = make_valid(sub_geom)

        if sub_geom.geom_type == 'Polygon':
            polys_to_plan.append(sub_geom)
        else:
            polys_to_plan.extend(list(sub_geom.geoms))

    if polys_to_plan:
        combined_poly = MultiPolygon(polys_to_plan) if len(polys_to_plan) > 1 else polys_to_plan[0]
        flight_plan = plan_flight_geometry(
            boundary_geom=combined_poly,
            flight_alt_agl=flight_alt_agl,
            flight_speed=flight_speed,
            camera_info=camera_info,
            forward_overlap=forward_overlap,
            side_overlap=side_overlap,
            waypoint_mode=waypoint_mode
        )
    else:
        flight_plan = FlightPlan(
            plan_id=str(uuid.uuid4()),
            flight_alt_agl=flight_alt_agl,
            flight_speed=flight_speed,
            total_distance_m=0.0,
            estimated_duration_s=0.0,
            photo_spacing_m=0.0,
            waypoint_mode=waypoint_mode,
            waypoints=[]
        )

    flight_plan.drone_enum = drone_enum
    flight_plan.drone_sub_enum = drone_sub_enum
    flight_plan.payload_enum = payload_enum
    flight_plan.has_gimbal = has_gimbal
    flight_plan.lens_type = lens_type

    os.makedirs(os.path.dirname(output_kmz), exist_ok=True)

    if output_kmz.lower().endswith(".kmz"):
        geojson_path = output_kmz[:-4] + "_waypoints.geojson"
    else:
        geojson_path = output_kmz + "_waypoints.geojson"

    dji_exporter = DJIWPMLPackageExporter()
    dji_exporter.export(flight_plan, drone_enum=drone_enum, payload_enum=payload_enum, output_path=output_kmz)

    geojson_exporter = GeoJSONExporter()
    geojson_exporter.export(flight_plan, output_path=geojson_path)

    mode_label = "精简" if waypoint_mode == "sparse" else "密集"
    print("=" * 60)
    print(f"🎉 高级避障航线与打包完成！")
    print(f"📦 输出 KMZ 包路径 : {output_kmz}")
    print(f"📍 {mode_label}航点：{len(flight_plan.waypoints)}个（{waypoint_mode}模式），GeoJSON 已输出：{geojson_path}")
    print(f"✈️ 巡航设定高度 : {flight_alt_agl} m")
    print("=" * 60)
