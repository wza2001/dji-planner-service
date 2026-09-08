from fastapi import FastAPI, HTTPException, BackgroundTasks, UploadFile, File, Form
from fastapi.responses import FileResponse
from app.models.schemas import PlannerRequest
from app.core.route_planner import plan_routes_from_safe_airspace
from app.core.parsers.factory import BoundaryParserFactory
import tempfile
import json
import os
import geopandas as gpd
from shapely.geometry import Polygon

app = FastAPI(title="DJI Route Planner API")

def remove_file(path: str):
    try:
        if path and os.path.exists(path):
            os.remove(path)
    except Exception:
        pass

def execute_planning_pipeline(polygon: Polygon, params: dict, background_tasks: BackgroundTasks) -> FileResponse:
    input_file = None
    output_file = None
    try:
        # Save polygon to a temporary geojson file for route planner
        tmp_in = tempfile.NamedTemporaryFile(delete=False, suffix=".geojson", mode="w")
        input_file = tmp_in.name
        tmp_in.close() # Close to avoid locks on Windows

        gdf = gpd.GeoDataFrame(index=[0], crs="EPSG:4326", geometry=[polygon])
        gdf.to_file(input_file, driver="GeoJSON")

        tmp_out = tempfile.NamedTemporaryFile(delete=False, suffix=".kmz")
        output_file = tmp_out.name
        tmp_out.close()

        camera_info = {
            "fov_horizontal_deg": params.get("fov_h", 84.0),
            "fov_vertical_deg": params.get("fov_v", 60.0)
        }

        # Call the core routing logic
        plan_routes_from_safe_airspace(
            safe_airspace_file=input_file,
            flight_alt_agl=params.get("agl", 120.0),
            flight_speed=params.get("speed", 10.0),
            camera_info=camera_info,
            forward_overlap=params.get("overlap_f", 0.8),
            side_overlap=params.get("overlap_s", 0.7),
            output_kmz=output_file,
            waypoint_mode=params.get("waypoint_mode", "sparse"),
            drone_enum=params.get("drone_enum", 95),
            payload_enum=params.get("payload_enum", 52)
        )

        if not os.path.exists(output_file):
            raise HTTPException(status_code=500, detail="Failed to generate KMZ file")

        # Cleanup input file
        remove_file(input_file)

        # Background tasks to clean up output files
        background_tasks.add_task(remove_file, output_file)
        geojson_out = output_file.replace('.kmz', '_waypoints.geojson')
        background_tasks.add_task(remove_file, geojson_out)

        return FileResponse(
            path=output_file,
            media_type="application/vnd.google-earth.kmz",
            filename="flight_routes.kmz"
        )
    except Exception as e:
        if input_file:
            remove_file(input_file)
        if output_file:
            remove_file(output_file)
        # Custom mapping could be applied here
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/v1/planner/generate_kmz")
def generate_kmz(request: PlannerRequest, background_tasks: BackgroundTasks):
    try:
        parser = BoundaryParserFactory.get_parser("geojson")
        poly = parser.parse(request.boundary_geojson)
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Geometry parsing error: {str(e)}")

    params = {
        "fov_h": request.fov_h,
        "fov_v": request.fov_v,
        "agl": request.agl,
        "speed": request.speed,
        "overlap_f": request.overlap_f,
        "overlap_s": request.overlap_s,
        "drone_enum": request.drone_enum,
        "payload_enum": request.payload_enum,
        "waypoint_mode": request.waypoint_mode
    }

    return execute_planning_pipeline(poly, params, background_tasks)

@app.post("/api/v1/planner/generate_kmz_from_file")
async def generate_kmz_from_file(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(..., description="Upload .kml or .geojson file"),
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
    ext = file.filename.split(".")[-1]
    if not ext:
        raise HTTPException(status_code=400, detail="Filename must have an extension")

    try:
        parser = BoundaryParserFactory.get_parser(ext)
    except ValueError as ve:
         raise HTTPException(status_code=400, detail=str(ve))

    content = await file.read()

    try:
        poly = parser.parse(content)
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"File parsing error: {str(e)}")

    params = {
        "fov_h": fov_h,
        "fov_v": fov_v,
        "agl": agl,
        "speed": speed,
        "overlap_f": overlap_f,
        "overlap_s": overlap_s,
        "drone_enum": drone_enum,
        "payload_enum": payload_enum,
        "waypoint_mode": waypoint_mode
    }

    return execute_planning_pipeline(poly, params, background_tasks)
