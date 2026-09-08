import json
from typing import Union
from shapely.geometry import shape, Polygon, MultiPolygon
from shapely.validation import make_valid
import shapely
from app.core.parsers.base import BaseBoundaryParser

class GeoJSONParser(BaseBoundaryParser):
    def parse(self, data: Union[str, bytes, dict]) -> Polygon:
        if isinstance(data, bytes):
            data = json.loads(data.decode('utf-8'))
        elif isinstance(data, str):
            data = json.loads(data)

        if data.get("type") == "FeatureCollection":
            features = data.get("features", [])
            if not features:
                raise ValueError("GeoJSON FeatureCollection must contain at least one Feature")
            geom = features[0].get("geometry")
        elif data.get("type") == "Feature":
            geom = data.get("geometry")
        else:
            geom = data

        if not geom:
            raise ValueError("No geometry found in GeoJSON")

        poly = shape(geom)

        if poly.has_z:
            poly = shapely.force_2d(poly)

        if isinstance(poly, MultiPolygon):
            poly = max(poly.geoms, key=lambda p: p.area)
        elif not isinstance(poly, Polygon):
            raise ValueError(f"Unsupported geometry type: {type(poly)}")

        if not poly.is_valid:
            poly = make_valid(poly)
            if hasattr(poly, 'geoms'):
                poly = max((p for p in poly.geoms if isinstance(p, Polygon)), key=lambda p: p.area, default=None)
            if not isinstance(poly, Polygon):
                raise ValueError("make_valid failed to return a valid Polygon")

        return poly
