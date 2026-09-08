import json
from typing import Union
from shapely.geometry import shape, Polygon
from app.sore.parsers.base import BaseBoundaryParser

class GeoJsonParser(BaseBoundaryParser):
    def parse(self, data: Union[str, bytes, dict]) -> Polygon:
        if isinstance(data, (str, bytes)):
            data = json.loads(data)

            #兼容FeatureCollection，Feature，或者Polygon
        if data.get("type") == "FeatureCollection":
            features = data.get("features", [])
            if not features:
                raise ValueError("GeoJSON FeatureCollection must contain at least one Feature")
            geom = features[0].get("geometry")
        elif data.get("type") == "Feature":
            geom = data.get("geometry")
        else:
            geom = data

        poly = shape(geom)
        if not poly.is_valid:
            poly = poly.buffer(0) #自动修复自相交等无效多边形
        return poly
