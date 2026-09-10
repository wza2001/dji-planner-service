# Task 05: API Dual-Track Input & Hardware Parameter Resolution

## Objective
Update FastAPI request schemas and API endpoints to support dual-track camera input: pre-configured DJI/PSDK models via identifiers, or custom 3rd-party cameras (Sony, Phase One) via physical optical parameters.

## Target Files
- Modify: `app/models/schemas.py`
- Modify: `app/main.py`

## Requirements
1. **Schema Evolution (`app/models/schemas.py`)**:
   - Update `PlannerRequest`:
     - Keep backward compatibility for raw `fov_h`, `fov_v` if provided.
     - Add preset identification: `drone_model: Optional[str] = "matrice_350_rtk"`, `camera_model: Optional[str] = "zenmuse_p1_35mm"`.
     - Add custom 3rd-party camera fields:
       - `is_custom_camera: bool = False`
       - `sensor_width_mm: Optional[float] = None`
       - `sensor_height_mm: Optional[float] = None`
       - `focal_length_mm: Optional[float] = None`
       - `has_gimbal: Optional[bool] = True`
       - `lens_type: Optional[str] = "single"`
     - Implement Pydantic validator: If `is_custom_camera` is True, require `sensor_width_mm`, `sensor_height_mm`, and `focal_length_mm`.

2. **Endpoint Updates in `app/main.py`**:
   - Support both `/api/v1/planner/generate_kmz` and `/api/v1/planner/generate_kmz_from_file`.
   - Update form parameters in `generate_kmz_from_file` to accept `drone_model`, `camera_model`, or custom sensor parameters.
   - Resolution pipeline before invoking planner:
     - Query `DeviceRegistry`:
       - If preset: resolve `drone_enum`, `payload_enum`, `fov_h`, `fov_v`, `has_gimbal`, `lens_type`.
       - If custom: resolve via `DeviceRegistry.build_custom_payload(...)`.
     - Pass the resolved hardware spec into `plan_routes_from_safe_airspace`.

## Acceptance Criteria
- Calling `/api/v1/planner/generate_kmz` with `{"drone_model": "matrice_350_rtk", "camera_model": "share_102s_oblique", "boundary_geojson": {...}}` automatically derives `fov_h`, `fov_v`, `has_gimbal=False` without requiring manual FOV input.
- Calling with custom sensor specs calculates correct FOV and generates a valid KMZ.