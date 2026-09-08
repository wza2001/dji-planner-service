from abc import ABC, abstractclassmethod
from pathlib import Path
from typing import Union
from shapely.geometry import Polygon, MultiPolygon

class BaseBoundaryParser(ABC):
    """测区空间数据解析基类"""

    @abstractclassmethod
    def parse(self, input_file: Union[Path, str]) -> Union[Polygon, MultiPolygon]:
        """
        将输入数据解析并转换为标准的shapely polygon 或者 multipolygon (wgs74 projection)

        :param input_file: 文件二进制流，文本字符串或者字典
        :return:合法的shapely polygon
        """
        pass
