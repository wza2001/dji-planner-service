### Task 1: Implement Single-Building 180° Semi-Circle S-Scan Trajectory Engine

#### 1. Context & Objective
Create a new module `app/core/planner/facade_planner.py` to implement a 3D trajectory generation algorithm tailored for single-building facade photogrammetry and inspection. 
To prevent RC/video link loss caused by tall building signal occlusion, the planner executes a semi-circle S-scan (sweeping one facade at a time) with alternating odd/even tiers and a 3-stage collision-avoidance ladder transition between layers. 
Focus strictly on pure geometry, spatial coordinate math, and data modeling in this task (KMZ/WPML serialization will be handled in Task 2).

#### 2. Functional Requirements
1. **Boresight Vector & Sector Calculation**:
   - Given the building center $(X_{\text{center}}, Y_{\text{center}})$ and a ground station / view reference coordinate $(X_{\text{view}}, Y_{\text{view}})$, derive the boresight azimuth $\alpha_0$.
   - To guarantee photogrammetric tie-point stitching between front and back missions, expand the valid horizontal sweep angle to $[\alpha_0 - 95^\circ, \, \alpha_0 + 95^\circ]$ (total sweep span of $190^\circ$, providing a $10^\circ$ overlap margin on both side edges).
2. **Photogrammetric Sampling Step Math**:
   - Determine the standoff distance $D_{\text{standoff}}$ and flight radius $R = R_{\text{building}} + D_{\text{standoff}}$ based on the target face GSD (`face_gsd`) or user-configured standoff distance, focal length, and sensor specs.
   - Compute vertical layer step $\Delta H$ and layer count $M$ using vertical FOV (`fov_v`) and vertical overlap ratio (`overlap_v`):
     $$\Delta H = 2 \cdot D_{\text{standoff}} \cdot \tan\left(\frac{\text{FOV}_v}{2}\right) \cdot (1 - \text{overlap}_v)$$
   - Compute horizontal arc step $L_{\text{step}}$ and angular step $\Delta \theta$ using horizontal FOV (`fov_h`) and horizontal overlap ratio (`overlap_h`):
     $$L_{\text{step}} = 2 \cdot D_{\text{standoff}} \cdot \tan\left(\frac{\text{FOV}_h}{2}\right) \cdot (1 - \text{overlap}_h), \quad \Delta \theta = \frac{L_{\text{step}}}{R}$$
3. **Alternating S-Topology & 3-Stage Ladder Turnaround**:
   - **Even Layers ($2k$)**: Sweep clockwise from Point A ($\alpha_0 - 95^\circ$) to Point B ($\alpha_0 + 95^\circ$).
   - **Odd Layers ($2k+1$)**: Sweep counter-clockwise from Point B to Point A.
   - **Turnaround Ladder (No in-place vertical climbing)**:
     At the sweep endpoint, stop and execute a 3-stage transition:
     1. Radial Push-out: Translate radially outward by $3 \sim 5\,\text{m}$ (to $R + \Delta R$) to clear corner turbulence and obstacle avoidance buffers.
     2. Vertical Climb: Ascend vertically by $\Delta H$ at the outward position.
     3. Cut-in: Translate radially inward to reach the starting endpoint of the next layer.
4. **Approach & Exit Safe Anchors**:
   - Insert an **Approach Waypoint** before Layer 0: Located in open airspace along the $\alpha_0$ vector at distance $R + 15\,\text{m}$ and altitude $Z_{\text{start}}$.
   - Insert an **Exit Waypoint** after the top layer: Located along the $\alpha_0$ vector at height $Z_{\text{top}} + 15\,\text{m}$.

#### 3. Data Models
Define the following in `app/models/facade.py`:
- `PointType` (Enum): `APPROACH`, `ARC_ACTIVE`, `LADDER_OUT`, `LADDER_CLIMB`, `LADDER_IN`, `EXIT`.
- `TriggerAction` (Enum): `START_SHOOT`, `STOP_SHOOT`, `NONE`.
- `FacadeWaypoint`: `lat: float`, `lon: float`, `alt_wgs84: float`, `heading_deg: float`, `point_type: PointType`, `trigger_action: TriggerAction`.
- `FacadeTrajectoryResult`: `layers: int`, `total_distance: float`, `waypoints: List[FacadeWaypoint]`.

#### 4. Deliverables
- `app/models/facade.py`
- `app/core/planner/facade_planner.py`
- `app/test/test_facade_geometry.py`: Unit test validating layer distribution, $190^\circ$ sweep clamp, ladder push-out vectors, and approach/exit positions.