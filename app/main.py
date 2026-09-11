import os
import json
import tempfile
from typing import Optional
from fastapi import FastAPI, HTTPException, BackgroundTasks, UploadFile, File, Form
from fastapi.responses import FileResponse
from shapely.geometry import mapping

from app.models.schemas import PlannerRequest, FacadeRequest
from app.core.route_planner import plan_routes_from_safe_airspace
from app.core.parsers.factory import BoundaryParserFactory
from app.core.registry import DeviceRegistry
from app.models.hardware import LensType
from app.services.facade_service import generate_facade_kmz_bundle

# 1. 先实例化 FastAPI 应用
app = FastAPI(title="DJI Route Planner API")


def remove_file(path: str):
    try:
        if os.path.exists(path):
            os.remove(path)
    except Exception:
        pass


def resolve_hardware_params(
    drone_model: Optional[str],
    camera_model: Optional[str],
    is_custom_camera: bool,
    sensor_width_mm: Optional[float],
    sensor_height_mm: Optional[float],
    focal_length_mm: Optional[float],
    has_gimbal: Optional[bool],
    lens_type: Optional[str],
    default_drone_enum: int,
    default_payload_enum: int,
    default_fov_h: float,
    default_fov_v: float
) -> dict:
    drone_enum = default_drone_enum
    payload_enum = default_payload_enum
    fov_h = default_fov_h
    fov_v = default_fov_v
    final_has_gimbal = has_gimbal if has_gimbal is not None else True
    final_lens_type = lens_type if lens_type is not None else "single"

    try:
        if is_custom_camera:
            if sensor_width_mm and sensor_height_mm and focal_length_mm:
                payload_spec = DeviceRegistry.build_custom_payload(
                    name="custom_camera",
                    sensor_width_mm=sensor_width_mm,
                    sensor_height_mm=sensor_height_mm,
                    focal_length_mm=focal_length_mm,
                    has_gimbal=final_has_gimbal,
                    lens_type=LensType(final_lens_type)
                )
                fov_h = payload_spec.fov_h
                fov_v = payload_spec.fov_v
                final_has_gimbal = payload_spec.has_gimbal
                final_lens_type = payload_spec.lens_type.value
                payload_enum = payload_spec.payload_enum

            if drone_model:
                drone_spec = DeviceRegistry.get_drone(drone_model)
                drone_enum = drone_spec.drone_enum
        else:
            if drone_model:
                drone_spec = DeviceRegistry.get_drone(drone_model)
                drone_enum = drone_spec.drone_enum

            if camera_model:
                payload_spec = DeviceRegistry.get_payload(camera_model)
                payload_enum = payload_spec.payload_enum
                fov_h = payload_spec.fov_h
                fov_v = payload_spec.fov_v
                final_has_gimbal = payload_spec.has_gimbal
                final_lens_type = payload_spec.lens_type.value
    except Exception as e:
        print(f"Warning: Failed to resolve hardware parameters from registry: {e}")

    return {
        "drone_enum": drone_enum,
        "payload_enum": payload_enum,
        "fov_h": fov_h,
        "fov_v": fov_v,
        "has_gimbal": final_has_gimbal,
        "lens_type": final_lens_type
    }

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

        hw_params = resolve_hardware_params(
            drone_model=request.drone_model,
            camera_model=request.camera_model,
            is_custom_camera=request.is_custom_camera,
            sensor_width_mm=request.sensor_width_mm,
            sensor_height_mm=request.sensor_height_mm,
            focal_length_mm=request.focal_length_mm,
            has_gimbal=request.has_gimbal,
            lens_type=request.lens_type,
            default_drone_enum=request.drone_enum,
            default_payload_enum=request.payload_enum,
            default_fov_h=request.fov_h,
            default_fov_v=request.fov_v
        )

        camera_info = {
            "fov_horizontal_deg": hw_params["fov_h"],
            "fov_vertical_deg": hw_params["fov_v"]
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
            drone_enum=hw_params["drone_enum"],
            payload_enum=hw_params["payload_enum"],
            has_gimbal=hw_params["has_gimbal"],
            lens_type=hw_params["lens_type"]
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
    waypoint_mode: str = Form("sparse"),
    drone_model: Optional[str] = Form("matrice_350_rtk"),
    camera_model: Optional[str] = Form("zenmuse_p1_35mm"),
    is_custom_camera: bool = Form(False),
    sensor_width_mm: Optional[float] = Form(None),
    sensor_height_mm: Optional[float] = Form(None),
    focal_length_mm: Optional[float] = Form(None),
    has_gimbal: Optional[bool] = Form(True),
    lens_type: Optional[str] = Form("single")
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

        hw_params = resolve_hardware_params(
            drone_model=drone_model,
            camera_model=camera_model,
            is_custom_camera=is_custom_camera,
            sensor_width_mm=sensor_width_mm,
            sensor_height_mm=sensor_height_mm,
            focal_length_mm=focal_length_mm,
            has_gimbal=has_gimbal,
            lens_type=lens_type,
            default_drone_enum=drone_enum,
            default_payload_enum=payload_enum,
            default_fov_h=fov_h,
            default_fov_v=fov_v
        )

        camera_info = {
            "fov_horizontal_deg": hw_params["fov_h"],
            "fov_vertical_deg": hw_params["fov_v"]
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
            drone_enum=hw_params["drone_enum"],
            payload_enum=hw_params["payload_enum"],
            has_gimbal=hw_params["has_gimbal"],
            lens_type=hw_params["lens_type"]
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

@app.post("/api/v1/planner/generate_facade_kmz")
def generate_facade_kmz(request: FacadeRequest, background_tasks: BackgroundTasks):
    try:
        output_file = generate_facade_kmz_bundle(request)
        background_tasks.add_task(remove_file, output_file)

        return FileResponse(
            path=output_file,
            media_type="application/zip",
            filename="building_facade_inspection.zip"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Planning error: {str(e)}")