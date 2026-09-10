import io
import zipfile
import xml.etree.ElementTree as ET
from typing import Union, List, Tuple
from shapely.geometry import Polygon, MultiPolygon
from app.core.parsers.base import BaseBoundaryParser


class KMLParser(BaseBoundaryParser):
    def parse(self, data: Union[str, bytes, dict]) -> Union[Polygon, MultiPolygon]:
        if isinstance(data, str):
            content = data.encode("utf-8")
        elif isinstance(data, bytes):
            content = data
        else:
            raise TypeError("KML 解析器只接受字符串或二进制文件流")

        # 1. 自动识别并解压 KMZ (ZIP 格式头 PK\x03\x04)
        if content.startswith(b"PK\x03\x04"):
            with zipfile.ZipFile(io.BytesIO(content), "r") as z:
                kml_names = [n for n in z.namelist() if n.lower().endswith(".kml")]
                if not kml_names:
                    raise ValueError("KMZ 包内未找到任何有效的 .kml 文件")
                content = z.read(kml_names[0])

        # 2. 解析 XML
        try:
            root = ET.fromstring(content)
        except Exception as e:
            raise ValueError(f"XML 解析失败，非合法 KML 文件: {e}")

        # 3. 提取所有 Polygon 要素（忽略 XML 命名空间）
        polygons: List[Polygon] = []

        for elem in root.iter():
            tag_name = elem.tag.split("}")[-1]
            if tag_name == "Polygon":
                exterior_coords = None
                interior_coords = []

                for child in elem.iter():
                    child_tag = child.tag.split("}")[-1]
                    if child_tag == "outerBoundaryIs":
                        for sub in child.iter():
                            if sub.tag.split("}")[-1] == "coordinates" and sub.text:
                                exterior_coords = self._extract_coords(sub.text)
                    elif child_tag == "innerBoundaryIs":
                        for sub in child.iter():
                            if sub.tag.split("}")[-1] == "coordinates" and sub.text:
                                holes = self._extract_coords(sub.text)
                                if len(holes) >= 3:
                                    interior_coords.append(holes)

                if exterior_coords and len(exterior_coords) >= 3:
                    poly = Polygon(shell=exterior_coords, holes=interior_coords)
                    if not poly.is_valid:
                        poly = poly.buffer(0)
                    polygons.append(poly)

        if not polygons:
            raise ValueError("KML 中未找到有效的 Polygon/LinearRing 边界要素")

        if len(polygons) == 1:
            return polygons[0]
        return MultiPolygon(polygons)

    @staticmethod
    def _extract_coords(coord_text: str) -> List[Tuple[float, float]]:
        """从 KML 坐标文本 (lon,lat,alt lon,lat,alt ...) 提取 (lon, lat) 列表"""
        coords = []
        for token in coord_text.strip().split():
            parts = token.split(",")
            if len(parts) >= 2:
                try:
                    lon = float(parts[0])
                    lat = float(parts[1])
                    coords.append((lon, lat))
                except ValueError:
                    continue
        return coords