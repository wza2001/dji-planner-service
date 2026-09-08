from typing import Dict, Type
from app.core.parsers.base import BaseBoundaryParser
from app.core.parsers.geojson_parser import GeoJSONParser
from app.core.parsers.kml_parser import KMLParser

class BoundaryParserFactory:
    _parser: Dict[str, Type[BaseBoundaryParser]] = {
        "geojson": GeoJSONParser,
        "json": GeoJSONParser,
        "kml": KMLParser,
        #未来扩展只需要在这里添加相应工具即可，例如：
        #"shape"：ShapefileParser
        #"zip": ShapefileParser
    }

    @classmethod
    def get_parser(cls, format_type: str) -> Type[BaseBoundaryParser]:
        fmt = format_type.lower().strip().replace(" ", "_")
        parser_cls = cls._parser.get(fmt)
        if not parser_cls:
            supported = ",".join(cls._parser.keys())
            raise ValueError(f"Unknown parser type: {fmt}")
        return parser_cls
