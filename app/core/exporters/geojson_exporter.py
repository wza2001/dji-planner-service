import json
from typing import Union, Dict, Any
from app.core.exporters.base import BaseFlightPlanExporter
from app.models.flight_plan import FlightPlan

class GeoJSONExporter(BaseFlightPlanExporter):
    def export(self, plan: FlightPlan, **kwargs) -> Union[Dict[str, Any], str]:
        output_path = kwargs.get('output_path', None)
        features = []
        for wp in plan.waypoints:
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [wp.lon, wp.lat, wp.alt]
                },
                "properties": {
                    "index": wp.index,
                    "type": wp.type,
                    "speed": wp.speed,
                    "heading_in": wp.heading_in,
                    "heading_out": wp.heading_out,
                    "heading_change": wp.heading_change,
                    "gimbal_pitch": wp.gimbal_pitch,
                    "segment_idx": wp.segment_idx
                }
            }
            features.append(feature)
        feature_collection = {
            "type": "FeatureCollection",
            "features": features
        }
        if output_path:
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(feature_collection, f, indent=2, ensure_ascii=False)
            return output_path
        return feature_collection
