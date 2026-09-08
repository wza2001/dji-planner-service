# Role & Objective
You are an expert Python/FastAPI backend engineer and geospatial software architect. 
Your task is twofold:
1. **Audit and harden** our spatial boundary parsing module (`app/core/parsers/`) designed to parse GeoJSON and KML (and extensible to Shapefile/Zip in the future) into standard Shapely geometries.
2. **Refactor and update `app/main.py`** to integrate this parsing architecture cleanly, preserving backward compatibility for existing JSON requests while adding a new `multipart/form-data` file upload endpoint.

---

## Part 1: Code Audit & Hardening of `app/core/parsers/`

Review and audit the proposed parser implementations (`base.py`, `geojson_parser.py`, `kml_parser.py`, `factory.py`). Specifically address and fix the following geospatial and operational edge cases:

1. **GDAL/OGR Resource Cleanup & Safety (`kml_parser.py`)**:
   - In GDAL Python bindings, opening a datasource (`ogr.Open`) locks system handles. Ensure `ds = None` and layer references are explicitly dereferenced in a `finally` block before deleting any temporary files on Windows and Linux.
   - Prevent OGR memory leaks and avoid unhandled C++ exceptions by catching OGR parsing failures cleanly and raising Python `ValueError`.

2. **Spatial Reference System (CRS) & Coordinate Order**:
   - GeoJSON is strictly EPSG:4326 with `[Longitude, Latitude]` order.
   - For KML, inspect whether the geometry contains a Spatial Reference. If it is not WGS84 (EPSG:4326), reproject it to EPSG:4326 using `osgeo.osr.CoordinateTransformation`.
   - Strip 3D coordinates ($Z$ values) if present (e.g., KML coordinates often contain `lon,lat,alt`), or ensure the planar Shapely `Polygon` handles them cleanly without breaking 2D route-planning assumptions.

3. **MultiPolygon & Complex Geometry Handling**:
   - Real-world KML/GeoJSON files frequently contain `MultiPolygon` or multiple features.
   - If a `MultiPolygon` is received, extract the largest polygon by area or explode/merge it safely, raising an informative `ValueError` if disjoint non-contiguous polygons cannot be routed together.
   - Ensure `poly.buffer(0)` or `shapely.validation.make_valid` is applied safely to self-intersecting rings.

4. **Parser Factory Extensibility (`factory.py`)**:
   - Ensure the factory handles varied file extensions (e.g., `.kml`, `.geojson`, `.json`, case-insensitive).
   - Keep the structure open for registering future parsers (e.g., `ShapefileParser` for `.zip`/`.shp`).

---

## Part 2: `app/main.py` Integration & Endpoint Implementation

Refactor `app/main.py` to integrate the audited parsers with the following requirements:

1. **Unified Planning Pipeline Execution**:
   - Extract the shared execution logic into an internal helper function:
     `execute_planning_pipeline(polygon: Polygon, params: dict, background_tasks: BackgroundTasks) -> FileResponse`
   - Ensure this helper manages:
     - Calling the core route planner with the validated Shapely polygon.
     - Generating the DJI-compliant KMZ archive.
     - Registering `BackgroundTasks` to automatically delete all generated temporary files after streaming the response.
     - Catching custom exceptions and mapping them to appropriate HTTP status codes (e.g., 400 for invalid geometry/parameters, 500 for internal calculation failures).

2. **Refactor Existing JSON Endpoint (Preserve Backward Compatibility)**:
   - Endpoint: `POST /api/v1/planner/generate_kmz`
   - Input: Pydantic model `PlannerRequest` (JSON payload with `boundary_geojson`).
   - Implementation: Parse `payload.boundary_geojson` through `BoundaryParserFactory.get_parser("geojson")`, obtain the Shapely polygon, and forward it to `execute_planning_pipeline`.

3. **Implement New File Upload Endpoint**:
   - Endpoint: `POST /api/v1/planner/generate_kmz_from_file`
   - Consumes: `multipart/form-data`
   - Form parameters:
     - `file`: `UploadFile = File(..., description="Upload .kml or .geojson file")`
     - Flight configuration parameters matching `PlannerRequest` fields as `Form(...)` with sensible defaults:
       `fov_h` (float, 84.0), `fov_v` (float, 60.0), `agl` (float, 120.0), `speed` (float, 10.0),
       `overlap_f` (float, 0.8), `overlap_s` (float, 0.7), `drone_enum` (int, 95),
       `payload_enum` (int, 52), `waypoint_mode` (str, "sparse").
   - Implementation:
     - Validate the filename extension.
     - Read the file bytes asynchronously (`await file.read()`).
     - Resolve the parser via `BoundaryParserFactory.get_parser(ext)`.
     - Parse the binary content into a valid Shapely polygon.
     - Delegate to `execute_planning_pipeline`.

---

## Part 3: Deliverables Required

Provide your output in the following structure:
1. **Audit Summary**: Bullet points explaining what vulnerabilities, edge cases, or resource leaks were identified in the preliminary parser logic and how you resolved them.
2. **Complete Code for Parser Module**:
   - `app/core/parsers/base.py`
   - `app/core/parsers/geojson_parser.py`
   - `app/core/parsers/kml_parser.py`
   - `app/core/parsers/factory.py`
3. **Updated `app/main.py`**: The complete production-ready file including imports, schemas, endpoints, and background cleanup logic.
4. **Curl / Test Snippets**: Example `curl` commands demonstrating how to test both the JSON endpoint and the `multipart/form-data` file upload endpoint.