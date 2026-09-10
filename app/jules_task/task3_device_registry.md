# Task 03: Device Registry & Hardware Specification Model

## Objective
Establish a configuration-driven Device Registry to decouple drone and camera hardware specs from route planning logic. Provide automated calculation of horizontal and vertical FOV based on sensor dimensions and focal length.

## Target Files
- Create: `app/configs/devices.yaml`
- Create: `app/models/hardware.py`
- Create: `app/core/registry.py`

## Requirements
1. **Configuration File (`app/configs/devices.yaml`)**:
   - `drones`: Define pre-configured drones with `drone_enum` and `drone_sub_enum`:
     - `matrice_350_rtk` (drone_enum: 89)
     - `matrice_300_rtk` (drone_enum: 60)
     - `mavic_3_enterprise` (drone_enum: 77)
   - `payloads`: Define pre-configured cameras with physical specs:
     - `zenmuse_p1_35mm`: `payload_enum: 50`, `has_gimbal: true`, `lens_type: "single"`, `sensor_width_mm: 35.9`, `sensor_height_mm: 24.0`, `focal_length_mm: 35.0`
     - `share_102s_oblique`: `payload_enum: 65535` (PSDK), `has_gimbal: false`, `lens_type: "oblique_5lens"`, `sensor_width_mm: 23.5`, `sensor_height_mm: 15.6`, `focal_length_mm: 25.0`
     - `mavic_3e_wide`: `payload_enum: 52`, `has_gimbal: true`, `lens_type: "single"`, `sensor_width_mm: 17.3`, `sensor_height_mm: 13.0`, `focal_length_mm: 12.3`

2. **Data Models (`app/models/hardware.py`)**:
   - Define Enums: `LensType` (`single`, `oblique_5lens`, `multi_spectrum`).
   - Define `PayloadSpec(BaseModel)`:
     - Fields: `name`, `payload_enum`, `has_gimbal`, `lens_type`, `sensor_width_mm`, `sensor_height_mm`, `focal_length_mm`, `fixed_pitch_deg` (optional, default -90.0).
     - Method or Property: Calculate `fov_h` and `fov_v` in degrees:
       `fov = 2 * math.degrees(math.atan(sensor_dimension / (2 * focal_length)))`
   - Define `DroneSpec(BaseModel)`:
     - Fields: `name`, `drone_enum`, `drone_sub_enum`, `supported_payload_ids: list[str]`.

3. **Registry Manager (`app/core/registry.py`)**:
   - Class `DeviceRegistry`:
     - Singleton or cached loader reading `app/configs/devices.yaml`.
     - Methods: `get_drone(drone_id: str) -> DroneSpec`, `get_payload(payload_id: str) -> PayloadSpec`.
     - Method `build_custom_payload(...) -> PayloadSpec`: Instantiates a `PayloadSpec` for 3rd-party cameras (e.g. Sony/PhaseOne) using user-supplied sensor and focal length values.

## Acceptance Criteria
- Run unit test: `python -c "from app.core.registry import DeviceRegistry; p = DeviceRegistry.get_payload('zenmuse_p1_35mm'); assert round(p.fov_h, 1) == 54.3"`
- Ensure `devices.yaml` parses without error and FOV calculation matches physical optics formulas.