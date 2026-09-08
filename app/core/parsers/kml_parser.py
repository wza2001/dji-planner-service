import tempfile
import os
from typing import Union
from osgeo import ogr, osr
from shapely import wkt
from shapely.geometry import Polygon, MultiPolygon
from shapely.validation import make_valid
import shapely
from app.core.parsers.base import BaseBoundaryParser

class KMLParser(BaseBoundaryParser):
    def parse(self, data: Union[str, bytes, dict]) -> Polygon:
        if isinstance(data, str):
            content = data.encode('utf-8')
        elif isinstance(data, bytes):
            content = data
        else:
            raise TypeError("KML 解析器只接受字符串或者二进制文件流")

        tmp_path = None
        ds = None
        layer = None
        try:
            tmp = tempfile.NamedTemporaryFile(suffix=".kml", delete=False)
            tmp.write(content)
            tmp_path = tmp.name
            tmp.close()

            ds = ogr.Open(tmp_path)
            if not ds:
                raise ValueError("无法解析该kml文件，文件格式不符合OGR kml标准")

            layer = ds.GetLayer(0)
            if not layer:
                raise ValueError("KML文件中找不到图层")

            spatial_ref = layer.GetSpatialRef()
            transform = None
            if spatial_ref:
                # Check if it's not WGS84
                target_srs = osr.SpatialReference()
                target_srs.ImportFromEPSG(4326)
                if not spatial_ref.IsSame(target_srs):
                    # We might need to handle axes order for EPSG:4326 depending on gdal version.
                    # Commonly OGR defaults to Traditional GIS order for KML but just in case:
                    target_srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
                    transform = osr.CoordinateTransformation(spatial_ref, target_srs)

            poly = None
            for feature in layer:
                geom = feature.GetGeometryRef()
                if not geom:
                    continue

                if transform:
                    geom.Transform(transform)

                geom_name = geom.GetGeometryName()
                if geom_name in ["POLYGON", "MULTIPOLYGON"]:
                    poly = wkt.loads(geom.ExportToWkt())
                    break

            if poly is None:
                raise ValueError("KML 中未找到合法的POLYGON边界要素")

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

        finally:
            if layer is not None:
                layer = None
            if ds is not None:
                ds = None
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass
