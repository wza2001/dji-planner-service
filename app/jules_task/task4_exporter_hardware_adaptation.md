# Task 04: Hardware-Aware WPML Exporter (Gimbal & Multi-Lens Control)

## Objective
Refactor `DJIWPMLPackageExporter` in `app/core/exporters/dji_exporter.py` to dynamically adjust WPML actions and tags according to hardware capabilities (gimbal vs. rigid mount, single lens vs. 5-lens oblique).

## Target Files
- Modify: `app/models/flight_plan.py`
- Modify: `app/core/exporters/dji_exporter.py`

## Requirements
1. **Flight Plan Hardware Context (`app/models/flight_plan.py`)**:
   - Ensure `FlightPlan` has access to hardware metadata (pass `drone_spec: DroneSpec` and `payload_spec: PayloadSpec`, or explicit flags: `has_gimbal: bool`, `lens_type: str`, `drone_enum: int`, `payload_enum: int`).

2. **Gimbal Logic in `DJIWPMLPackageExporter`**:
   - Check `has_gimbal`:
     - **When `has_gimbal == True`**:
       - Inject `<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>` at route entry / waypoints.
       - Set `<wpml:gimbalPitchRotateAngle>` to payload target pitch (e.g., -90 for nadir, -45 for oblique).
     - **When `has_gimbal == False` (Fixed mount)**:
       - **Strictly omit** any `gimbalRotate` or `gimbalPitchRotateAngle` action elements from `waylines.wpml`.
       - In `<wpml:waypointHeadingParam>`, set `<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>` to ensure the aircraft body aligns with the flight path.

3. **Multi-Lens / Oblique Support**:
   - For `lens_type == "oblique_5lens"` (e.g., PSDK cameras):
     - Trigger standard `takePhoto` action (`<wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>`).
     - Do not generate DJI-proprietary `payloadLensIndex` elements to avoid PSDK rejection.

4. **Template & Waylines Header**:
   - Inject dynamic `droneEnumValue`, `droneSubEnumValue`, `payloadEnumValue`, `payloadPositionIndex: 0` into both `template.kml` and `waylines.wpml`.

## Acceptance Criteria
- Exporting a flight plan with `has_gimbal=False` produces a KMZ whose internal `waylines.wpml` contains zero occurrences of `<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>`.
- Exporting a flight plan with `has_gimbal=True` retains correct gimbal pitch commands.