# Task: Decouple Path Planning Geometry from DJI WPML Exporter (Intermediate Representation Pattern)

## 1. Architectural Objective
Refactor the route planning pipeline in `dji-planner-service` to decouple hardware-agnostic geometric path planning from hardware-specific format serialization (DJI WPML/KMZ). 
We introduce an **Intermediate Representation (IR)** schema (`FlightPlan`) and an **Exporter Strategy Pattern** (`BaseFlightPlanExporter`). The geometric path planning engine must output pure, structured flight plan objects, while dedicated exporter modules translate them into DJI KMZ packages or GeoJSON.

---

## 2. Hard Constraints & Boundary Rules (MANDATORY)
1. **Directory Immutability**:
   - The project root is `dji-planner-service/`.
   - All source code MUST reside under `app/`. Do NOT move existing folders (`core/`, `models/`, `utils/`, `parsers/`) out of `app/`.
   - Do NOT restructure existing project-level configuration files (`Dockerfile`, `docker-compose.dev.yml`, `requirements.txt`).
2. **Backward Compatibility**:
   - Existing API endpoints in `app/main.py` (`POST /api/v1/planner/generate_kmz` and `POST /api/v1/planner/generate_kmz_from_file`) must remain fully functional with identical HTTP request/response contracts.
3. **No External Microservices**:
   - Keep all changes self-contained within this repository. Do not introduce new network protocols or external services.

---

## 3. Implementation Steps & File Specifications

### Step 1: Define Intermediate Representation (IR) Schema
Create `app/models/flight_plan.py`:
- `GenericWaypoint`: Pydantic model with fields:
  - `index: int`
  - `lon: float`
  - `lat: float`
  - `alt: float`
  - `speed: float`
  - `type: str` ("takeoff", "scan_start", "scan_end", "turn", "transit", "landing")
  - `heading_in: float = 0.0`
  - `heading_out: float = 0.0`
  - `heading_change: float = 0.0`
  - `gimbal_pitch: float = -90.0`
  - `segment_idx: int = -1`
- `FlightPlan`: Pydantic model with fields:
  - `plan_id: str`
  - `flight_alt_agl: float`
  - `flight_speed: float`
  - `total_distance_m: float`
  - `estimated_duration_s: float`
  - `photo_spacing_m: float`
  - `waypoint_mode: str` ("sparse" | "dense")
  - `waypoints: list[GenericWaypoint]`
  - Helper method `to_dict() -> dict`

### Step 2: Define Exporter Abstraction
Create `app/core/exporters/base.py`:
- Define abstract base class `BaseFlightPlanExporter(ABC)`:
  - `@abstractmethod def export(self, plan: FlightPlan, **kwargs) -> Any`: Abstract method to export the plan into targeted vendor formats.

### Step 3: Implement DJI WPML Package Exporter
Create `app/core/exporters/dji_exporter.py`:
- Move and refactor the XML formatting and packaging functions currently inside `app/core/route_planner.py`:
  - `generate_dji_template_kml(plan: FlightPlan, drone_enum: int, payload_enum: int) -> str`
  - `generate_dji_waylines_wpml(plan: FlightPlan, drone_enum: int, payload_enum: int) -> str`
  - `package_dji_kmz_bytes(template_kml: str, waylines_wpml: str) -> bytes` (Use in-memory `io.BytesIO` + `zipfile` instead of mandatory disk writes)
- Implement `DJIWPMLPackageExporter(BaseFlightPlanExporter)`:
  - Inherits from `BaseFlightPlanExporter`.
  - Accepts `drone_enum: int` and `payload_enum: int` via `kwargs`.
  - Supports exporting directly to in-memory `bytes` (for FastAPI streaming) or writing to a file path.

### Step 4: Implement GeoJSON Exporter
Create `app/core/exporters/geojson_exporter.py`:
- Implement `GeoJSONExporter(BaseFlightPlanExporter)`:
  - Converts `FlightPlan.waypoints` into a standard `GeoJSON FeatureCollection` (Points with properties `index`, `type`, `heading_in`, `heading_out`, `turn_mode`, etc.).
  - Returns a GeoJSON `dict` or writes to a `.geojson` file.

### Step 5: Refactor Core Route Planning Algorithm
Refactor `app/core/route_planner.py`:
- Retain geometric algorithms: `VisibilityRouter`, `classify_and_enrich_waypoints`, `generate_grid_flight_path`, and `generate_dense_grid_flight_path`.
- Extract the core planning execution into a pure function:
  ```python
  def plan_flight_geometry(
      boundary_geom: Polygon | MultiPolygon,
      flight_alt_agl: float = 120.0,
      flight_speed: float = 10.0,
      camera_info: dict = None,
      forward_overlap: float = 0.80,
      side_overlap: float = 0.70,
      waypoint_mode: str = "sparse"
  ) -> FlightPlan