from typing import Dict, Type
from app.core.parsers.base import BaseBoundaryParser
from app.core.parsers.geojson_parser import GeoJSONParser
from app.core.parsers.kml_parser import KMLParser

class BoundaryParserFactory:
    _parsers: Dict[str, Type[BaseBoundaryParser]] = {
        "geojson": GeoJSONParser,
        "json": GeoJSONParser,
        "kml": KMLParser,
        # 未来扩展只需要在这里添加相应工具即可，例如：
        # "shape"：ShapefileParser
        # "zip": ShapefileParser
    }

    @classmethod
    def get_parser(cls, format_type: str) -> BaseBoundaryParser:
        fmt = format_type.lower().strip()
        # Remove a leading dot if present
        if fmt.startswith("."):
            fmt = fmt[1:]

        parser_cls = cls._parsers.get(fmt)
        if not parser_cls:
            supported = ", ".join(cls._parsers.keys())
            raise ValueError(f"Unknown parser type: {format_type}. Supported types: {supported}")
        return parser_cls()
