import io
import zipfile
import datetime
import math
import os
from typing import Union
from app.models.facade import FacadeTrajectoryResult, PointType, TriggerAction

def generate_facade_template_kml(plan: FacadeTrajectoryResult, center_lat: float, center_lon: float,
                                drone_enum: int, payload_enum: int, speed: float,
                                has_gimbal: bool, view_lat: float, view_lon: float) -> str:
    total_distance = plan.total_distance
    # estimated duration
    duration = total_distance / speed if speed > 0 else 0
    height_mode = "WGS84"
    current_time = datetime.datetime.now().isoformat()

    placemark_nodes = []

    # 1. Takeoff Reference Point (just for map display)
    placemark_nodes.append(f"""
    <Placemark>
      <Point>
        <coordinates>{view_lon:.6f},{view_lat:.6f}</coordinates>
      </Point>
      <wpml:index>0</wpml:index>
      <wpml:executeHeight>0.0</wpml:executeHeight>
      <wpml:waypointSpeed>0</wpml:waypointSpeed>
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>{center_lon:.6f},{center_lat:.6f},0.000000</wpml:waypointPoiPoint>
      </wpml:waypointHeadingParam>
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>toPointAndStopWithDiscontinuityCurvature</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>1</wpml:useStraightLine>
    </Placemark>""")

    for i, wp in enumerate(plan.waypoints):
        # We start indices at 1 for the actual path
        wp_index = i + 1
        lon = wp.lon
        lat = wp.lat
        alt = wp.alt_wgs84

        turn_mode = "toPointAndStopWithDiscontinuityCurvature"
        use_straight_line = 1
        if wp.point_type == PointType.ARC_ACTIVE:
            turn_mode = "toPointAndPassWithContinuityCurvature"
            use_straight_line = 0

        gimbal_pitch_xml = ""
        if has_gimbal:
            # simple tiered pitch based on altitude...
            # In template.kml, we can omit complex actionGroups, they go in waylines.
            gimbal_pitch = -30.0 # placeholder
            gimbal_pitch_xml = f"\n      <wpml:gimbalPitchAngle>{gimbal_pitch}</wpml:gimbalPitchAngle>"

        placemark_xml = f"""
    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{wp_index}</wpml:index>
      <wpml:executeHeight>{alt:.1f}</wpml:executeHeight>
      <wpml:waypointSpeed>{speed}</wpml:waypointSpeed>
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>{center_lon:.6f},{center_lat:.6f},0.000000</wpml:waypointPoiPoint>
      </wpml:waypointHeadingParam>
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>{use_straight_line}</wpml:useStraightLine>{gimbal_pitch_xml}
    </Placemark>"""
        placemark_nodes.append(placemark_xml)

    placemarks_xml = "".join(placemark_nodes)

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <name>waylines</name>
    <wpml:createTime>{current_time}</wpml:createTime>
    <wpml:updateTime>{current_time}</wpml:updateTime>
    <wpml:missionConfig>
      <wpml:flyToWaylineMode>safely</wpml:flyToWaylineMode>
      <wpml:finishAction>goHome</wpml:finishAction>
      <wpml:exitOnRCLost>goContinue</wpml:exitOnRCLost>
      <wpml:executeRCLostAction>goBack</wpml:executeRCLostAction>
      <wpml:globalTransitionalSpeed>{speed}</wpml:globalTransitionalSpeed>
      <wpml:globalRTHHeight>100</wpml:globalRTHHeight>
      <wpml:droneInfo>
        <wpml:droneEnumValue>{drone_enum}</wpml:droneEnumValue>
        <wpml:droneSubEnumValue>0</wpml:droneSubEnumValue>
      </wpml:droneInfo>
      <wpml:payloadInfo>
        <wpml:payloadEnumValue>{payload_enum}</wpml:payloadEnumValue>
        <wpml:payloadSubEnumValue>0</wpml:payloadSubEnumValue>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadInfo>
    </wpml:missionConfig>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:distance>{total_distance:.2f}</wpml:distance>
      <wpml:duration>{duration:.2f}</wpml:duration>
      <wpml:autoFlightSpeed>{speed}</wpml:autoFlightSpeed>
      <wpml:executeHeightMode>{height_mode}</wpml:executeHeightMode>
      <wpml:payloadParam>
        <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
      </wpml:payloadParam>
      <wpml:takeOffRefPoint>{view_lon:.6f},{view_lat:.6f},0.0</wpml:takeOffRefPoint>{placemarks_xml}
    </Folder>
  </Document>
</kml>"""


def generate_facade_waylines_wpml(plan: FacadeTrajectoryResult, center_lat: float, center_lon: float,
                                  speed: float, has_gimbal: bool, l_step_m: float) -> str:
    height_mode = "WGS84"
    current_time = datetime.datetime.now().isoformat()
    placemark_nodes = []

    group_id = 0

    # helper for finding next LADDER_OUT
    def get_end_index_for_shooting(start_idx):
        for i in range(start_idx, len(plan.waypoints)):
            if plan.waypoints[i].trigger_action == TriggerAction.STOP_SHOOT:
                return i + 1
        return len(plan.waypoints)

    for i, wp in enumerate(plan.waypoints):
        wp_index = i + 1
        lon = wp.lon
        lat = wp.lat
        alt = wp.alt_wgs84

        turn_mode = "toPointAndStopWithDiscontinuityCurvature"
        use_straight_line = 1
        if wp.point_type == PointType.ARC_ACTIVE:
            turn_mode = "toPointAndPassWithContinuityCurvature"
            use_straight_line = 0

        action_groups_xml = ""

        if wp.trigger_action == TriggerAction.START_SHOOT:
            end_index = get_end_index_for_shooting(i)

            gimbal_action = ""
            if has_gimbal:
                # Calculate tiered pitch: simple approximation based on layer / altitude
                pitch = -15
                if alt > 40: pitch = -35
                if alt > 70: pitch = -55

                gimbal_action = f"""
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>{pitch}</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
            <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>"""

                # Gimbal initialization at the start of layer
                action_groups_xml += f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>{group_id}</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp_index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp_index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
      </wpml:actionGroup>"""
                group_id += 1

            # Multiple distance shooting
            action_groups_xml += f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>{group_id}</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp_index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{end_index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
          <wpml:actionTriggerParam>{l_step_m}</wpml:actionTriggerParam>
        </wpml:actionTrigger>
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:fileSuffix>photo</wpml:fileSuffix>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>
      </wpml:actionGroup>"""
            group_id += 1



        gimbal_pitch_xml = ""
        #if has_gimbal:
        #    gimbal_pitch_xml = f"\n      <wpml:gimbalPitchAngle>-30.0</wpml:gimbalPitchAngle>"

        placemark_xml = f"""
    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{wp_index}</wpml:index>
      <wpml:executeHeight>{alt:.1f}</wpml:executeHeight>
      <wpml:waypointSpeed>{speed}</wpml:waypointSpeed>
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
        <wpml:waypointPoiPoint>{center_lon:.6f},{center_lat:.6f},0.000000</wpml:waypointPoiPoint>
      </wpml:waypointHeadingParam>
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>{use_straight_line}</wpml:useStraightLine>{gimbal_pitch_xml}{action_groups_xml}
    </Placemark>"""
        placemark_nodes.append(placemark_xml)

    placemarks_xml = "".join(placemark_nodes)

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:wpml="http://www.dji.com/wpmz/1.0.2">
  <Document>
    <name>waylines</name>
    <wpml:createTime>{current_time}</wpml:createTime>
    <wpml:updateTime>{current_time}</wpml:updateTime>
    <Folder>
      <wpml:templateId>0</wpml:templateId>
      <wpml:waylineId>0</wpml:waylineId>
      <wpml:templateType>waypoint</wpml:templateType>
      <wpml:executeHeightMode>{height_mode}</wpml:executeHeightMode>
      <wpml:waylinesInfo>
        <wpml:waylineId>0</wpml:waylineId>
        <wpml:useGlobalHeight>0</wpml:useGlobalHeight>
      </wpml:waylinesInfo>{placemarks_xml}
    </Folder>
  </Document>
</kml>"""


def package_facade_kmz(plan: FacadeTrajectoryResult, center_lat: float, center_lon: float,
                       drone_enum: int, payload_enum: int, speed: float,
                       has_gimbal: bool, view_lat: float, view_lon: float, l_step_m: float) -> bytes:

    template_kml = generate_facade_template_kml(plan, center_lat, center_lon, drone_enum, payload_enum, speed, has_gimbal, view_lat, view_lon)
    waylines_wpml = generate_facade_waylines_wpml(plan, center_lat, center_lon, speed, has_gimbal, l_step_m)

    mem_zip = io.BytesIO()
    with zipfile.ZipFile(mem_zip, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.writestr('wpmz/template.kml', template_kml.encode('utf-8'))
        kmz.writestr('wpmz/waylines.wpml', waylines_wpml.encode('utf-8'))
    return mem_zip.getvalue()
