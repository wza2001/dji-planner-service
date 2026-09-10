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


def get_turn_mode(heading_in: float, heading_out: float) -> str:
    """
    根据进入/离开航向变化量选择 DJI WPML 转弯模式字符串。

    - 变化 < 10°  : smoothTransition
    - 变化 < 45°  : toPointAndStopWithContinuityCurvature
    - 变化 >= 45° : toPointAndStopWithDiscontinuityCurvature
    """
    diff = abs(heading_out - heading_in)
    change = min(diff, 360.0 - diff)
    if change < 10:
        return "smoothTransition"
    elif change < 45:
        return "toPointAndStopWithContinuityCurvature"
    else:
        return "toPointAndStopWithDiscontinuityCurvature"


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


def generate_dji_template_kml(waypoints: list = None, drone_enum: int = 68, payload_enum: int = 52, speed: float = 12) -> str:
    """
    生成 DJI WPML 标准 template.kml 字符串（任务元数据）。

    Args:
        waypoints: 航点列表。
        drone_enum: 无人机型号枚举值，默认 68（Mavic 3 Enterprise）。
        payload_enum: 负载型号枚举值，默认 52（H20T）。
        speed: 全局过渡速度（m/s）。

    Returns:
        template.kml 的 XML 字符串。
    """
    create_time = int(time.time() * 1000)

    placemarks_xml = ""
    if waypoints:
        is_dict = isinstance(waypoints[0], dict)
        nodes = []
        for idx in range(len(waypoints)):
            if is_dict:
                wp = waypoints[idx]
                lon, lat = wp["lon"], wp["lat"]
            else:
                lon, lat = waypoints[idx][0], waypoints[idx][1]
            nodes.append(f'''    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{idx}</wpml:index>
      <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
      <wpml:useGlobalSpeed>1</wpml:useGlobalSpeed>
      <wpml:useGlobalHeadingParam>1</wpml:useGlobalHeadingParam>
      <wpml:useGlobalTurnParam>1</wpml:useGlobalTurnParam>
    </Placemark>''')
        placemarks_xml = "\n".join(nodes)

    return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <wpml:createTime>{create_time}</wpml:createTime>
    <wpml:updateTime>{create_time}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{speed}</wpml:globalTransitionalSpeed>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{drone_enum}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>{payload_enum}</wpml:payloadEnumValue>
        <wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:waylineCoordinateSysParam>
        <wpml:coordinateMode>WGS84</wpml:coordinateMode>
        <wpml:heightMode>relativeToStartPoint</wpml:heightMode>
      </wpml:waylineCoordinateSysParam>
      <wpml:autoFlightSpeed>{speed}</wpml:autoFlightSpeed>
      <wpml:globalHeight>60.0</wpml:globalHeight>
      <wpml:globalWaypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
      </wpml:globalWaypointHeadingParam>
      <wpml:globalWaypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:globalWaypointTurnMode>
      <wpml:globalUseStraightLine>1</wpml:globalUseStraightLine>
{placemarks_xml}
    </Folder>
  </Document>
</kml>'''


def generate_dji_waylines_wpml(waypoints: list,
                               flight_speed: float = 10,
                               flight_alt: float = 120,
                               height_mode: str = "relativeToStartPoint",
                               photo_spacing_m: float = 20.0,
                               drone_enum: int = 68,
                               payload_enum: int = 52) -> str:
    """
    生成 DJI WPML 标准 waylines.wpml 字符串（航点执行细节）。

    结构严格遵循 DJI WPML 1.0.2：
      Document -> Folder -> Placemark -> Point + wpml:waypoint + actionGroup

    兼容两种输入：
      - list[dict]（sparse 模式，新精简航点）：仅起飞点配置动作组
        （云台朝下 + 全局 multipleDistance 触发拍照），其余航点无动作组，
        转弯模式按航向动态计算。
      - list[tuple]（dense 模式，旧版密集航点）：逐点配置云台与等距拍照动作，
        保持历史行为。

    Args:
        waypoints: 航点列表（list[dict] 或 list[tuple[float, float, float]]）。
        flight_speed: 自动飞行速度（m/s）。
        flight_alt: 飞行相对高度（m）。
        height_mode: 高度模式，默认 relativeToStartPoint。
        photo_spacing_m: 等距拍照间隔（米），用于 multipleDistance 触发。

    Returns:
        waylines.wpml 的 XML 字符串。
    """
    if not waypoints:
        return ""

    # 判断输入类型：list[dict] (sparse) 还是 list[tuple] (dense)
    is_dict = isinstance(waypoints[0], dict)
    last_idx = len(waypoints) - 1

    # ── 计算总距离和总时长 ──
    total_distance = 0.0
    seg_distances = [0.0]  # 每段距离，seg_distances[i] = waypoints[i-1] -> waypoints[i]
    for i in range(1, len(waypoints)):
        if is_dict:
            lon_i = waypoints[i]["lon"]
            lat_i = waypoints[i]["lat"]
            lon_prev = waypoints[i - 1]["lon"]
            lat_prev = waypoints[i - 1]["lat"]
        else:
            lon_i, lat_i = waypoints[i][0], waypoints[i][1]
            lon_prev, lat_prev = waypoints[i - 1][0], waypoints[i - 1][1]

        dlon = lon_i - lon_prev
        dlat = lat_i - lat_prev
        # 使用 haversine 更精确
        lat_mid = math.radians((lat_i + lat_prev) / 2)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_mid) + 1.175 * math.cos(4 * lat_mid)
        m_per_deg_lon = 111412.84 * math.cos(lat_mid) - 93.5 * math.cos(3 * lat_mid)
        seg_m = math.hypot(dlon * m_per_deg_lon, dlat * m_per_deg_lat)
        seg_distances.append(seg_m)
        total_distance += seg_m

    duration = total_distance / flight_speed if flight_speed > 0 else 0

    # ── 生成每个航点的 Placemark ──
    placemark_nodes = []
    cumulative_dist = 0.0  # 累计飞行距离，用于 dense 模式的 multipleDistance 判断

    for idx in range(len(waypoints)):
        if is_dict:
            wp = waypoints[idx]
            lon = wp["lon"]
            lat = wp["lat"]
            alt = wp["alt"]
            wp_type = wp.get("type", "transit")
            turn_mode = get_turn_mode(wp.get("heading_in", 0.0), wp.get("heading_out", 0.0))
        else:
            lon, lat, alt = waypoints[idx]
            wp_type = "takeoff" if idx == 0 else ("landing" if idx == last_idx else "transit")
            turn_mode = "toPointAndStopWithDiscontinuityCurvature"

        cumulative_dist += seg_distances[idx]

        # 动作组构建
        action_groups_xml = ""

        if is_dict:
            # ── sparse 模式：仅起飞点配置动作组 ──
            if idx == 0:
                # 动作组 0：reachPoint 触发，云台朝下
                gimbal_action = f'''
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>-90</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
            <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
            <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>'''
                # 动作组 1：multipleDistance 触发，覆盖整个航线，统一等距拍照
                photo_action = f'''
        <wpml:action>
          <wpml:actionId>1</wpml:actionId>
          <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>'''
                action_groups_xml = f'''
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
      </wpml:actionGroup>
      <wpml:actionGroup>
        <wpml:actionGroupId>1</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{last_idx}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
          <wpml:actionTriggerParam>{photo_spacing_m:.2f}</wpml:actionTriggerParam>
        </wpml:actionTrigger>{photo_action}
      </wpml:actionGroup>'''
            else:
                # 中间航点 / landing：无动作组
                action_groups_xml = ""
        else:
            # ── dense 模式：保留旧版逐点动作逻辑 ──
            is_photo_point = (idx > 0) and (cumulative_dist >= photo_spacing_m)

            gimbal_action = f'''
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>-90</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
            <wpml:gimbalRollRotateEnable>0</wpml:gimbalRollRotateEnable>
            <wpml:gimbalYawRotateEnable>0</wpml:gimbalYawRotateEnable>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>'''

            photo_action = ""
            if is_photo_point:
                photo_action = f'''
        <wpml:action>
          <wpml:actionId>1</wpml:actionId>
          <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>'''

            if idx == 0:
                action_groups_xml = f'''
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
      </wpml:actionGroup>'''
            elif is_photo_point:
                action_groups_xml = f'''
      <wpml:actionGroup>
        <wpml:actionGroupId>{idx}</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{idx}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{idx}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
          <wpml:actionTriggerParam>{photo_spacing_m:.2f}</wpml:actionTriggerParam>
        </wpml:actionTrigger>{gimbal_action}{photo_action}
      </wpml:actionGroup>'''
            else:
                action_groups_xml = ""

        # 航点核心参数
        placemark_xml = f'''
    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{idx}</wpml:index>
      <wpml:executeHeight>{alt:.1f}</wpml:executeHeight>
      <wpml:waypointSpeed>{flight_speed}</wpml:waypointSpeed>
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
      </wpml:waypointHeadingParam>
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>0</wpml:useStraightLine>
      <wpml:gimbalPitchAngle>-90</wpml:gimbalPitchAngle>{action_groups_xml}
    </Placemark>'''
        placemark_nodes.append(placemark_xml)

    placemarks_xml = "".join(placemark_nodes)

    return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <name>waylines</name>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>executeLostAction</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{flight_speed}</wpml:globalTransitionalSpeed>
      <wpml:globalRTHHeight>100</wpml:globalRTHHeight>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{drone_enum}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>{payload_enum}</wpml:payloadEnumValue>
        <wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:distance>{total_distance:.2f}</wpml:distance>
      <wpml:duration>{duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{flight_speed}</wpml:autoFlightSpeed>
      <wpml:executeHeightMode>{height_mode}</wpml:executeHeightMode>
      <wpml:payloadParam>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadParam>{placemarks_xml}
    </Folder>
  </Document>
</kml>'''


def package_dji_kmz(template_kml_str: str, waylines_wpml_str: str, output_kmz_path: str) -> None:
    """
    将 DJI WPML 的 template.kml 与 waylines.wpml 打包为 KMZ。

    Args:
        template_kml_str: template.kml 的 XML 字符串。
        waylines_wpml_str: waylines.wpml 的 XML 字符串。
        output_kmz_path: 输出 KMZ 文件路径。
    """
    with zipfile.ZipFile(output_kmz_path, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.writestr('wpmz/template.kml', template_kml_str.encode('utf-8'))
        kmz.writestr('wpmz/waylines.wpml', waylines_wpml_str.encode('utf-8'))


def _wp_coord(wp):
    """统一从 dict 或 tuple 航点中提取 (lon, lat)。"""
    if isinstance(wp, dict):
        return wp["lon"], wp["lat"]
    return wp[0], wp[1]


def plan_routes_from_safe_airspace(safe_airspace_file: str,
                                   flight_alt_agl: float = 120.0,
                                   flight_speed: float = 10.0,
                                   camera_info: dict = None,
                                   forward_overlap: float = 0.80,
                                   side_overlap: float = 0.70,
                                   output_kmz: str = "output/flight_routes.kmz",
                                   waypoint_mode: str = "sparse",
                                   drone_enum: int = 68,
                                   payload_enum: int = 52):
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

    calc_res = calculate_flight_parameters(
        agl_m=flight_alt_agl, fov_h_deg=fov_h, fov_v_deg=fov_v,
        forward_overlap=forward_overlap, side_overlap=side_overlap,
        flight_speed_m_s=flight_speed
    )

    line_spacing_m = calc_res["line_spacing_m"]
    photo_spacing_m = calc_res.get("forward_spacing_m", 20.0)

    print(f"🔄 正在精算图论 DFS 与边界避障航线... (waypoint_mode={waypoint_mode})")
    success_count = 0
    all_waypoints = []

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

        sub_polys = [sub_geom] if isinstance(sub_geom, Polygon) else list(sub_geom.geoms)

        for p_idx, poly in enumerate(sub_polys):
            if poly.is_empty or poly.area < 1e-8:
                continue

            if waypoint_mode == "dense":
                # 旧版密集航点：返回 list[tuple]，逐点拍照
                waypoints = generate_dense_grid_flight_path(
                    boundary_geom=poly,
                    line_spacing_m=line_spacing_m,
                    photo_spacing_m=photo_spacing_m,
                    alt_agl=flight_alt_agl
                )
            else:
                # 默认 sparse：精简航点（list[dict]）
                waypoints = generate_grid_flight_path(
                    boundary_geom=poly,
                    line_spacing_m=line_spacing_m,
                    photo_spacing_m=photo_spacing_m,
                    alt_agl=flight_alt_agl
                )

            if not waypoints:
                continue

            all_waypoints.extend(waypoints)
            success_count += 1

    os.makedirs(os.path.dirname(output_kmz), exist_ok=True)

    # GeoJSON 输出路径：将 .kmz 后缀替换为 _waypoints.geojson
    if output_kmz.lower().endswith(".kmz"):
        geojson_path = output_kmz[:-4] + "_waypoints.geojson"
    else:
        geojson_path = output_kmz + "_waypoints.geojson"

    if all_waypoints:
        total_distance = sum(
            np.hypot(_wp_coord(all_waypoints[i])[0] - _wp_coord(all_waypoints[i - 1])[0],
                     _wp_coord(all_waypoints[i])[1] - _wp_coord(all_waypoints[i - 1])[1])
            for i in range(1, len(all_waypoints))
        ) * 111000  # 粗略转换为米

        duration = total_distance / flight_speed if flight_speed > 0 else 0

        template_str = generate_dji_template_kml(
            waypoints=all_waypoints,
            drone_enum=drone_enum,
            payload_enum=payload_enum,
            speed=flight_speed
        )
        waylines_str = generate_dji_waylines_wpml(
            waypoints=all_waypoints,
            flight_speed=flight_speed,
            flight_alt=flight_alt_agl,
            height_mode="relativeToStartPoint",
            photo_spacing_m=photo_spacing_m,
            drone_enum=drone_enum,
            payload_enum=payload_enum
        )
        package_dji_kmz(template_str, waylines_str, output_kmz)

        # GeoJSON 输出（sparse 模式为丰富属性的 dict；dense 模式转为基础 dict）
        if waypoint_mode == "sparse":
            geojson_waypoints = all_waypoints
        else:
            geojson_waypoints = [
                {
                    "lon": wp[0], "lat": wp[1], "alt": wp[2],
                    "type": "transit", "segment_idx": -1,
                    "heading_in": 0.0, "heading_out": 0.0, "heading_change": 0.0
                }
                for wp in all_waypoints
            ]
        save_waypoints_geojson(geojson_waypoints, geojson_path)
    else:
        # 无航点时生成空 KMZ，避免下游报错
        with zipfile.ZipFile(output_kmz, 'w', zipfile.ZIP_DEFLATED) as kmz:
            pass

    mode_label = "精简" if waypoint_mode == "sparse" else "密集"
    print("=" * 60)
    print(f"🎉 高级避障航线与打包完成！")
    print(f"📦 输出 KMZ 包路径 : {output_kmz}")
    print(f"📍 {mode_label}航点：{len(all_waypoints)}个（{waypoint_mode}模式），GeoJSON 已输出：{geojson_path}")
    print(f"✈️ 巡航设定高度 : {flight_alt_agl} m")
    print(f"🗺️ 成功生成高级航线板块数 : {success_count} 个")
    print("=" * 60)
