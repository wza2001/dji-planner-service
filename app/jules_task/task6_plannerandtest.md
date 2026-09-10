# Task 06: Geometric Planner Buffer Adaptation & Comprehensive Test Suite

## Objective
Optimize survey area buffering for oblique cameras in `route_planner.py` and write unit/integration tests covering the entire hardware registry and export pipeline.

## Target Files
- Modify: `app/core/route_planner.py`
- Create: `app/test/test_hardware_pipeline.py`

## Requirements
1. **Geometric Buffer Adaptation (`app/core/route_planner.py`)**:
   - Receive `lens_type: str = "single"` in `plan_routes_from_safe_airspace` (or via payload spec).
   - If `lens_type == "oblique_5lens"`:
     - Expand boundary buffer: Oblique side-looking lenses (45° tilt) require an additional safety buffer around the survey boundary to guarantee facade texture coverage. Increase outer polygon buffer by $H \times \tan(45^\circ) = H$ (where $H$ is flight altitude AGL) or a configurable factor.

2. **Comprehensive Test Suite (`app/test/test_hardware_pipeline.py`)**:
   - **Test 1 (`test_registry_calculations`)**:
     - Verify FOV calculation for standard full-frame 35mm lens ($35.9 \times 24.0$ mm sensor) equals approx $54.3^\circ$ horizontal and $37.8^\circ$ vertical.
   - **Test 2 (`test_fixed_mount_no_gimbal_export`)**:
     - Plan route with `share_102s_oblique` (`has_gimbal=False`).
     - Unzip generated KMZ and inspect `wpmz/waylines.wpml`.
     - Assert that `<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>` does NOT exist.
     - Assert that `<wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>` is present.
   - **Test 3 (`test_custom_camera_flow`)**:
     - Send a synthetic request with custom Phase One specs ($53.4 \times 40.0$ mm, 50mm focal length).
     - Assert 200 OK and valid KMZ returned.

## Acceptance Criteria
- Run `pytest app/test/test_hardware_pipeline.py` inside the container and ensure all tests pass (100% green).