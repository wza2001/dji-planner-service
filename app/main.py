import os
import json
import tempfile
from fastapi import FastAPI, HTTPException, BackgroundTasks, UploadFile, File, Form
from fastapi.responses import FileResponse
from shapely.geometry import mapping

from app.models.schemas import PlannerRequest
from app.core.route_planner import plan_routes_from_safe_airspace
from app.core.parsers.factory import BoundaryParserFactory

# 1. 先实例化 FastAPI 应用
app = FastAPI(title="DJI Route Planner API")


def remove_file(path: str):
    try:
        if os.path.exists(path):
            os.remove(path)
    except Exception:
        pass


# 2. 原有的 JSON 接口
@app.post("/api/v1/planner/generate_kmz")
def generate_kmz(request: PlannerRequest, background_tasks: BackgroundTasks):
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".geojson", mode="w", encoding="utf-8") as tmp_in:
            if isinstance(request.boundary_geojson, str):
                tmp_in.write(request.boundary_geojson)
            else:
                json.dump(request.boundary_geojson, tmp_in)
            input_file = tmp_in.name

        with tempfile.NamedTemporaryFile(delete=False, suffix=".kmz") as tmp_out:
            output_file = tmp_out.name

        camera_info = {
            "fov_horizontal_deg": request.fov_h,
            "fov_vertical_deg": request.fov_v
        }

        plan_routes_from_safe_airspace(
            safe_airspace_file=input_file,
            flight_alt_agl=request.agl,
            flight_speed=request.speed,
            camera_info=camera_info,
            forward_overlap=request.overlap_f,
            side_overlap=request.overlap_s,
            output_kmz=output_file,
            waypoint_mode=request.waypoint_mode,
            drone_enum=request.drone_enum,
            payload_enum=request.payload_enum
        )

        if not os.path.exists(output_file):
            raise HTTPException(status_code=500, detail="Failed to generate KMZ file")

        remove_file(input_file)
        background_tasks.add_task(remove_file, output_file)
        geojson_out = output_file.replace(".kmz", "_waypoints.geojson")
        background_tasks.add_task(remove_file, geojson_out)

        return FileResponse(
            path=output_file,
            media_type="application/vnd.google-earth.kmz",
            filename="flight_routes.kmz"
        )

    except Exception as e:
        if "input_file" in locals():
            remove_file(input_file)
        if "output_file" in locals():
            remove_file(output_file)
        raise HTTPException(status_code=500, detail=str(e))


# 3. 文件上传接口
@app.post("/api/v1/planner/generate_kmz_from_file")
async def generate_kmz_from_file(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(...),
    fov_h: float = Form(84.0),
    fov_v: float = Form(60.0),
    agl: float = Form(120.0),
    speed: float = Form(10.0),
    overlap_f: float = Form(0.8),
    overlap_s: float = Form(0.7),
    drone_enum: int = Form(95),
    payload_enum: int = Form(52),
    waypoint_mode: str = Form("sparse")
):
    tmp_geojson = None
    output_file = None
    try:
        # 1. 提取后缀并调用解析器工厂
        file_bytes = await file.read()
        file_ext = os.path.splitext(file.filename)[1].lstrip(".").lower() or "kml"
        parser_cls = BoundaryParserFactory.get_parser(file_ext)
        polygon = parser_cls().parse(file_bytes)

        # 2. 转换为 FeatureCollection 临时文件给规划器
        feature_collection = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "geometry": mapping(polygon),
                "properties": {"name": "Survey_Area"}
            }]
        }

        with tempfile.NamedTemporaryFile(delete=False, suffix=".geojson", mode="w", encoding="utf-8") as tmp_in:
            json.dump(feature_collection, tmp_in)
            tmp_geojson = tmp_in.name

        with tempfile.NamedTemporaryFile(delete=False, suffix=".kmz") as tmp_out:
            output_file = tmp_out.name

        camera_info = {
            "fov_horizontal_deg": fov_h,
            "fov_vertical_deg": fov_v
        }

        # 3. 规划航线
        plan_routes_from_safe_airspace(
            safe_airspace_file=tmp_geojson,
            flight_alt_agl=agl,
            flight_speed=speed,
            camera_info=camera_info,
            forward_overlap=overlap_f,
            side_overlap=overlap_s,
            output_kmz=output_file,
            waypoint_mode=waypoint_mode,
            drone_enum=drone_enum,
            payload_enum=payload_enum
        )

        if not os.path.exists(output_file):
            raise HTTPException(status_code=500, detail="Failed to generate KMZ file")

        # 4. 注册临时文件清理
        remove_file(tmp_geojson)
        background_tasks.add_task(remove_file, output_file)
        geojson_out = output_file.replace(".kmz", "_waypoints.geojson")
        background_tasks.add_task(remove_file, geojson_out)

        return FileResponse(
            path=output_file,
            media_type="application/vnd.google-earth.kmz",
            filename="flight_routes.kmz"
        )

    except Exception as e:
        if tmp_geojson:
            remove_file(tmp_geojson)
        if output_file:
            remove_file(output_file)
        raise HTTPException(status_code=500, detail=f"Planning error: {str(e)}")