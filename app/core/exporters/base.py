from abc import ABC, abstractmethod
from typing import Any
from app.models.flight_plan import FlightPlan

class BaseFlightPlanExporter(ABC):
    @abstractmethod
    def export(self, plan: FlightPlan, **kwargs) -> Any:
        pass
