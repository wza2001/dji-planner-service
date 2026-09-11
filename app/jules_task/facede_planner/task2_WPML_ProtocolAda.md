### Task 2: Extend DJI WPML Exporter for Facade S-Scan Missions

#### 1. Context & Objective
Extend the WPML generation logic (e.g., in `app/core/exporters/dji_exporter.py`) to convert `FacadeTrajectoryResult` into valid DJI WPML structures (`template.kml` and `waylines.wpml`). Ensure smooth curve flight on scan arcs, strict point-and-stop actions on ladder transitions, and correct payload attitude/shutter triggers.

#### 2. WPML Attribute Specifications
1. **Hybrid Curvature & Waypoint Turn Modes**:
   - For `ARC_ACTIVE` waypoints (active imaging arcs):
     - Set `<wpml:waypointTurnMode>toPointAndPassWithContinuityCurvature</wpml:waypointTurnMode>`
     - Set `<wpml:useStraightLine>0</wpml:useStraightLine>`
     Ensures uninterrupted, smooth trajectory and uniform ground speed during exposure.
   - For `LADDER_*`, `APPROACH`, and `EXIT` waypoints:
     - Set `<wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>`
     - Set `<wpml:useStraightLine>1</wpml:useStraightLine>`
     Prevents curve interpolation overshoot toward the building corners during turnaround.
2. **Shutter Action Groups**:
   - At the first `ARC_ACTIVE` waypoint of each layer: Inject an `actionGroup` with `actionTriggerType: multipleDistance` (interval set to $L_{\text{step}}$) to start equidistant shooting.
   - At the final `ARC_ACTIVE` waypoint of each layer: Inject an `actionGroup` with `stopShooting` action before entering the ladder climb, preventing blurry or redundant exposures during turnaround.
3. **Payload & Heading Strategy**:
   - **Gimbal single-lens cameras (`has_gimbal: true`, e.g., Zenmuse P1)**:
     - `<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>` with POI bound to building center.
     - Inject tiered `gimbalRotate` pitch angles based on altitude ($0^\circ \sim -15^\circ$ for lower levels, $-25^\circ \sim -35^\circ$ for mid-levels, $-50^\circ \sim -60^\circ$ near roof top).
   - **Rigid 5-lens oblique systems (`has_gimbal: false`, e.g., Riebo DG6P)**:
     - DO NOT inject any `gimbalRotate` actions.
     - Set `<wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>` pointed directly at the building center. The aircraft flies in a coordinated circle/yaw hold, keeping the nadir + 4 oblique lenses oriented properly with respect to the facade.
4. **Safety & Failsafe Policies**:
   - Height reference: Enforce `<wpml:executeHeightMode>WGS84</wpml:executeHeightMode>`.
   - RC Lost action: Set `<wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>` in `template.kml`. The aircraft must continue the planned arc back to the open side if link drops momentarily at corner margins, rather than attempting a blind direct RTH through the structure.
   - Finish action: `<wpml:finishAction>goHome</wpml:finishAction>`.

#### 3. Deliverables
- Updated WPML exporter files under `app/core/exporters/`.
- Unit test ensuring generated XML structures contain valid action groups, turn modes, and failsafe tags.
