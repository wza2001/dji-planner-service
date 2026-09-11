### Task 3: Dual-Mission (Front/Back) Orchestration & Facade API Endpoint

#### 1. Context & Objective
A full 3D reconstruction requires surveying both opposite sides of the building. Because continuous 360° flight cuts radio signal behind the structure, split the survey into two independent missions: `Face_A.kmz` (Front) and `Face_B.kmz` (Back). Expose a dedicated REST API returning a packaged ZIP bundle.

#### 2. Dual-Mission Orchestration Logic
1. **Boresight Pairing**:
   - Accept two reference viewpoint anchors (`viewpoint_a` and `viewpoint_b`). If only one viewpoint is provided, compute `viewpoint_b` as the exact $180^\circ$ symmetric opposite relative to the building center.
   - Generate two discrete trajectory sets:
     - `Mission_FaceA`: Covers $[\alpha_A - 95^\circ, \, \alpha_A + 95^\circ]$
     - `Mission_FaceB`: Covers $[\alpha_B - 95^\circ, \, \alpha_B + 95^\circ]$
2. **Takeoff Reference Handling (Non-locking)**:
   - Write `viewpoint_a` and `viewpoint_b` into the respective `<wpml:takeOffRefPoint>` nodes for map display reference only.
   - Enforce `<wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>` to ensure the aircraft climbs to the safe takeoff altitude before traversing to the Approach Waypoint.
3. **Artifact Packaging**:
   - Package `Face_A.kmz`, `Face_B.kmz`, and a metadata summary `mission_summary.json` (containing layer heights, estimated photo counts, overlap angles, and GSD) into a single ZIP archive.

#### 3. API Contract
Add the following endpoint in `app/main.py`:
- **Route**: `POST /api/v1/planner/generate_facade_kmz`
- **Request Form / Body**:
  - `building_geometry`: GeoJSON Polygon or WKT of the building perimeter footprint.
  - `ground_alt_m`: Base elevation (WGS84 ellipsoidal height).
  - `building_height_m`: Absolute building height above ground.
  - `viewpoint_a`: `{"lat": float, "lon": float}` (Recommended takeoff/line-of-sight zone for Face A).
  - `viewpoint_b`: Optional `{"lat": float, "lon": float}` (Recommended takeoff/line-of-sight zone for Face B).
  - `drone_model`: Default `"matrice_400"`.
  - `camera_model`: Default `"riebo_dg6p_oblique"`.
  - `face_gsd_cm`: Target face resolution (default `1.0`).
- **Response**: `application/zip` (`building_facade_inspection.zip`).

#### 4. Deliverables
- `app/services/facade_service.py`
- Updated `app/main.py` with the `/api/v1/planner/generate_facade_kmz` endpoint.
- Temporary file cleanup via FastAPI `BackgroundTasks`.