import io
import zipfile
import datetime
import os
from typing import Union
from app.core.exporters.base import BaseFlightPlanExporter
from app.models.flight_plan import FlightPlan

def get_turn_mode(heading_in: float, heading_out: float) -> str:
    """
    根据进入/离开航向变化量选择 DJI WPML 转弯模式字符串。

    - 变化 < 10°  : smoothTransition
    - 变化 < 45°  : toPointAndStopWithContinuityCurvature
    - 变化 >= 45° : toPointAndStopWithDiscontinuityCurvature
    """
    diff = abs(heading_out - heading_in)
    change = min(diff, 360.0 - diff)
    if change < 10:
        return "smoothTransition"
    elif change < 45:
        return "toPointAndStopWithContinuityCurvature"
    else:
        return "toPointAndStopWithDiscontinuityCurvature"




def generate_dji_template_kml(plan: FlightPlan) -> str:
    waypoints = plan.waypoints
    speed = plan.flight_speed
    drone_enum = plan.drone_enum
    drone_sub_enum = plan.drone_sub_enum
    payload_enum = plan.payload_enum

    placemark_nodes = []

    for wp in waypoints:
        wp_type = wp.type
        lon = wp.lon
        lat = wp.lat
        alt = wp.alt

        use_straight_line = "0"
        if wp_type == 'ARC_ACTIVE':
            turn_mode = 'toPointAndPassWithContinuityCurvature'
        elif wp_type in ['LADDER_OUT', 'LADDER_CLIMB', 'LADDER_IN', 'APPROACH', 'EXIT']:
            turn_mode = 'toPointAndStopWithDiscontinuityCurvature'
            use_straight_line = "1"
        elif wp_type in ['takeoff', 'scan_start']:
            turn_mode = 'coordinateTurn'
        elif wp_type == 'scan_end':
            turn_mode = 'stopAndTurn'
        elif wp_type == 'transit':
            turn_mode = get_turn_mode(wp.heading_in, wp.heading_out)
        else:
            turn_mode = 'coordinateTurn'

        action_groups_xml = ""
        if plan.has_gimbal and wp_type == 'scan_start':
            action_groups_xml = f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp.index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>{wp.gimbal_pitch}</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
            <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>
      </wpml:actionGroup>"""

        gimbal_pitch_xml = f"\n      <wpml:gimbalPitchAngle>{wp.gimbal_pitch}</wpml:gimbalPitchAngle>" if plan.has_gimbal else ""


        if wp_type in ['ARC_ACTIVE', 'LADDER_OUT', 'LADDER_CLIMB', 'LADDER_IN', 'APPROACH', 'EXIT']:
            heading_param = f"""
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>
        <wpml:waypointPoiPoint>{wp.poi_lon:.6f},{wp.poi_lat:.6f},{wp.poi_alt:.1f}</wpml:waypointPoiPoint>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
      </wpml:waypointHeadingParam>"""
        else:
            heading_param = f"""
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
      </wpml:waypointHeadingParam>"""

        placemark_xml = f"""
    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{wp.index}</wpml:index>
      <wpml:executeHeight>{alt:.1f}</wpml:executeHeight>
      <wpml:waypointSpeed>{wp.speed}</wpml:waypointSpeed>{heading_param}
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>{use_straight_line}</wpml:useStraightLine>{gimbal_pitch_xml}{action_groups_xml}
    </Placemark>"""
        placemark_nodes.append(placemark_xml)

    placemarks_xml = "".join(placemark_nodes)

    total_distance = plan.total_distance_m
    duration = plan.estimated_duration_s
    height_mode = "WGS84"

    current_time = datetime.datetime.now().isoformat()

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
        <wpml:droneSubEnumValue>{drone_sub_enum}</wpml:droneSubEnumValue>
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
      </wpml:payloadParam>{placemarks_xml}
    </Folder>
  </Document>
</kml>"""

def generate_dji_waylines_wpml(plan: FlightPlan) -> str:
    waypoints = plan.waypoints
    height_mode = "WGS84"
    photo_spacing_m = plan.photo_spacing_m
    drone_enum = plan.drone_enum
    drone_sub_enum = plan.drone_sub_enum
    payload_enum = plan.payload_enum

    placemark_nodes = []

    for wp in waypoints:
        wp_type = wp.type
        lon = wp.lon
        lat = wp.lat
        alt = wp.alt

        use_straight_line = "0"
        if wp_type == 'ARC_ACTIVE':
            turn_mode = 'toPointAndPassWithContinuityCurvature'
        elif wp_type in ['LADDER_OUT', 'LADDER_CLIMB', 'LADDER_IN', 'APPROACH', 'EXIT']:
            turn_mode = 'toPointAndStopWithDiscontinuityCurvature'
            use_straight_line = "1"
        elif wp_type in ['takeoff', 'scan_start']:
            turn_mode = 'coordinateTurn'
        elif wp_type == 'scan_end':
            turn_mode = 'stopAndTurn'
        elif wp_type == 'transit':
            turn_mode = get_turn_mode(wp.heading_in, wp.heading_out)
        else:
            turn_mode = 'coordinateTurn'

        action_groups_xml = ""

        # Facade S-Scan logic for ARC_ACTIVE
        if wp_type == 'ARC_ACTIVE':
            # Check if first or last ARC_ACTIVE in the layer
            wp_idx = waypoints.index(wp)
            is_first = (wp_idx == 0 or waypoints[wp_idx-1].type != 'ARC_ACTIVE')
            is_last = (wp_idx == len(waypoints)-1 or waypoints[wp_idx+1].type != 'ARC_ACTIVE')

            end_index = wp.index
            if is_first:
                # find end of this ARC_ACTIVE sequence
                for s_wp in waypoints[wp_idx:]:
                    if s_wp.type != 'ARC_ACTIVE':
                        break
                    end_index = s_wp.index

            gimbal_action = f"""
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>{wp.gimbal_pitch}</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
            <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>""" if plan.has_gimbal else ""

            if is_first:
                action_groups_xml = f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp.index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
      </wpml:actionGroup>
      <wpml:actionGroup>
        <wpml:actionGroupId>1</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{end_index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
          <wpml:actionTriggerParam>{photo_spacing_m}</wpml:actionTriggerParam>
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
            elif is_last:
                action_groups_xml = f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>2</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp.index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>stopShooting</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>
      </wpml:actionGroup>"""
        if plan.waypoint_mode == "sparse":
            if wp_type == 'scan_start':
                end_index = len(waypoints) - 1 # Default to last
                for s_wp in waypoints[wp.index:]:
                    if s_wp.type == 'scan_end':
                        end_index = s_wp.index
                        break

                gimbal_action = f"""
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>{wp.gimbal_pitch}</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
            <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>""" if plan.has_gimbal else ""

                action_groups_xml = f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp.index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
      </wpml:actionGroup>
      <wpml:actionGroup>
        <wpml:actionGroupId>1</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{end_index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>multipleDistance</wpml:actionTriggerType>
          <wpml:actionTriggerParam>{photo_spacing_m}</wpml:actionTriggerParam>
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
        else: # dense 模式
            gimbal_action = f"""
        <wpml:action>
          <wpml:actionId>0</wpml:actionId>
          <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:gimbalPitchRotateAngle>{wp.gimbal_pitch}</wpml:gimbalPitchRotateAngle>
            <wpml:gimbalRollRotateAngle>0</wpml:gimbalRollRotateAngle>
            <wpml:gimbalYawRotateAngle>0</wpml:gimbalYawRotateAngle>
            <wpml:gimbalRotateTimeEnable>0</wpml:gimbalRotateTimeEnable>
            <wpml:gimbalRotateTime>0</wpml:gimbalRotateTime>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>""" if plan.has_gimbal else ""

            action_groups_xml = f"""
      <wpml:actionGroup>
        <wpml:actionGroupId>0</wpml:actionGroupId>
        <wpml:actionGroupStartIndex>{wp.index}</wpml:actionGroupStartIndex>
        <wpml:actionGroupEndIndex>{wp.index}</wpml:actionGroupEndIndex>
        <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
        <wpml:actionTrigger>
          <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
        </wpml:actionTrigger>{gimbal_action}
        <wpml:action>
          <wpml:actionId>1</wpml:actionId>
          <wpml:actionActuatorFunc>takePhoto</wpml:actionActuatorFunc>
          <wpml:actionActuatorFuncParam>
            <wpml:fileSuffix>photo</wpml:fileSuffix>
            <wpml:payloadPositionIndex>0</wpml:payloadPositionIndex>
          </wpml:actionActuatorFuncParam>
        </wpml:action>
      </wpml:actionGroup>"""

        gimbal_pitch_xml = f"\n      <wpml:gimbalPitchAngle>{wp.gimbal_pitch}</wpml:gimbalPitchAngle>" if plan.has_gimbal else ""


        if wp_type in ['ARC_ACTIVE', 'LADDER_OUT', 'LADDER_CLIMB', 'LADDER_IN', 'APPROACH', 'EXIT']:
            heading_param = f"""
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>towardPOI</wpml:waypointHeadingMode>
        <wpml:waypointPoiPoint>{wp.poi_lon:.6f},{wp.poi_lat:.6f},{wp.poi_alt:.1f}</wpml:waypointPoiPoint>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
      </wpml:waypointHeadingParam>"""
        else:
            heading_param = f"""
      <wpml:waypointHeadingParam>
        <wpml:waypointHeadingMode>followWayline</wpml:waypointHeadingMode>
        <wpml:waypointHeadingAngle>0</wpml:waypointHeadingAngle>
      </wpml:waypointHeadingParam>"""

        placemark_xml = f"""
    <Placemark>
      <Point>
        <coordinates>{lon:.6f},{lat:.6f}</coordinates>
      </Point>
      <wpml:index>{wp.index}</wpml:index>
      <wpml:executeHeight>{alt:.1f}</wpml:executeHeight>
      <wpml:waypointSpeed>{wp.speed}</wpml:waypointSpeed>{heading_param}
      <wpml:waypointTurnParam>
        <wpml:waypointTurnMode>{turn_mode}</wpml:waypointTurnMode>
        <wpml:waypointTurnDampingDist>0</wpml:waypointTurnDampingDist>
      </wpml:waypointTurnParam>
      <wpml:useStraightLine>{use_straight_line}</wpml:useStraightLine>{gimbal_pitch_xml}{action_groups_xml}
    </Placemark>"""
        placemark_nodes.append(placemark_xml)

    placemarks_xml = "".join(placemark_nodes)

    current_time = datetime.datetime.now().isoformat()

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
        <wpml:useGlobalHeight>1</wpml:useGlobalHeight>
      </wpml:waylinesInfo>{placemarks_xml}
    </Folder>
  </Document>
</kml>"""

def package_dji_kmz_bytes(template_kml_str: str, waylines_wpml_str: str) -> bytes:
    mem_zip = io.BytesIO()
    with zipfile.ZipFile(mem_zip, 'w', zipfile.ZIP_DEFLATED) as kmz:
        kmz.writestr('wpmz/template.kml', template_kml_str.encode('utf-8'))
        kmz.writestr('wpmz/waylines.wpml', waylines_wpml_str.encode('utf-8'))
    return mem_zip.getvalue()

class DJIWPMLPackageExporter(BaseFlightPlanExporter):
    def export(self, plan: FlightPlan, **kwargs) -> Union[bytes, str]:
        output_path = kwargs.get('output_path', None)

        if not plan.waypoints:
            mem_zip = io.BytesIO()
            with zipfile.ZipFile(mem_zip, 'w', zipfile.ZIP_DEFLATED) as kmz:
                pass
            b = mem_zip.getvalue()
        else:
            template_kml_str = generate_dji_template_kml(plan)
            waylines_wpml_str = generate_dji_waylines_wpml(plan)
            b = package_dji_kmz_bytes(template_kml_str, waylines_wpml_str)

        if output_path:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, 'wb') as f:
                f.write(b)
            return output_path
        return b
