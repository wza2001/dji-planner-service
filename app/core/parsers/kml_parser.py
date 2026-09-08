import tempfile
import os
from typing import Union
from osgeo import ogr
from shapely import wkt
from shapely.geometry import polygon
from app.core.parsers.base import BaseBoundaryParser

class KmlParser(BaseBoundaryParser):
    def parse(self, data: Union[str, bytes, dict]) -> Polygon:
        if isinstance(data, str):
            content = data.encode('utf-8')
        elif isinstance(data, bytes):
            content = data
        else:
            raise TypeError("KML 解析器只接受字符串或者二进制文件流")

        #使用内存临时文件交由GDAL/OGR 驱动解析
        with tempfile.NamedTemporaryFile(suffix=".kml", delete=False) as tmp:
            temp.write(content)
            tmp_path = tmp.name
        try:
            ds = ogs.Open(tmp_path)
            if not ds:
                raise ValueError("无法解析该kml文件，文件格式不符合OGR kml标准")
            layer = ds.GetLayer(0)
            poly = None
            for feature in layer:
                geom = feature.GetGeometryRef()
                if geom.GetGeometryName() in ["POLYGON", "MULTIPOLYGON"]:
                    poly = wkt.loads(geom.ExportToWkt())
                    break
            if poly is None:
                raise ValueError("KML 中未找到合法的POLYGON边界要素")
            if not poly.is_valid:
                poly = poly.buffer(0)
            return poly

        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
