### Task 4: End-to-End Test Suite for M400 + Riebo DG6P Facade Missions

#### 1. Context & Objective
Create an end-to-end integration test `app/test/test_facade_m400_riebo.py` to validate the entire workflow using a simulated 80-meter tall building with Matrice 400 and Riebo DG6P 5-lens oblique payload.

#### 2. Test Verification & Assertions
1. **Archive Integrity**:
   - Send mock payload with building geometry and two viewpoints to `POST /api/v1/planner/generate_facade_kmz`.
   - Confirm HTTP 200 response with `Content-Type: application/zip`.
   - Inspect ZIP contents and assert presence of `Face_A.kmz`, `Face_B.kmz`, and `mission_summary.json`.
2. **Hardware Registry Mapping**:
   - Parse `template.kml` and `waylines.wpml` inside each KMZ.
   - Assert `<wpml:droneEnumValue>103</wpml:droneEnumValue>` (Matrice 400).
   - Assert `<wpml:payloadEnumValue>65535</wpml:payloadEnumValue>` (Riebo PSDK).
   - Assert NO `<wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>` tags exist anywhere in the files (protection for rigid 5-lens systems).
3. **Flight Trajectory & Turn Modes**:
   - Assert both `toPointAndPassWithContinuityCurvature` (arc segments) and `toPointAndStopWithDiscontinuityCurvature` (ladder transitions) are present.
   - Assert `<wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>` is set.
   - Assert shutter trigger mode uses `multipleDistance`.
4. **Wrap-Around Overlap**:
   - Verify the angular span of the sweep waypoints in each mission is $\approx 190^\circ$, ensuring corner tie-point overlap.

#### 3. Deliverables
- `app/test/test_facade_m400_riebo.py`
- Execute the script from the host environment to verify all assertions pass cleanly.