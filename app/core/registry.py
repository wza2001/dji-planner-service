import yaml
import os
from app.models.hardware import DroneSpec, PayloadSpec, LensType

class DeviceRegistry:
    _instance = None
    _config = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DeviceRegistry, cls).__new__(cls)
            cls._instance._load_config()
        return cls._instance

    def _load_config(self):
        config_path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'devices.yaml')
        with open(config_path, 'r') as f:
            self._config = yaml.safe_load(f)

    @classmethod
    def get_drone(cls, drone_id: str) -> DroneSpec:
        registry = cls()
        drones_config = registry._config.get('drones', {})
        if drone_id not in drones_config:
            raise ValueError(f"Drone {drone_id} not found in configuration")
        return DroneSpec(**drones_config[drone_id])

    @classmethod
    def get_payload(cls, payload_id: str) -> PayloadSpec:
        registry = cls()
        payloads_config = registry._config.get('payloads', {})
        if payload_id not in payloads_config:
            raise ValueError(f"Payload {payload_id} not found in configuration")
        return PayloadSpec(**payloads_config[payload_id])

    @classmethod
    def build_custom_payload(cls, name: str, sensor_width_mm: float, sensor_height_mm: float, focal_length_mm: float,
                             payload_enum: int = 65535, has_gimbal: bool = False, lens_type: LensType = LensType.single,
                             fixed_pitch_deg: float = -90.0) -> PayloadSpec:
        return PayloadSpec(
            name=name,
            payload_enum=payload_enum,
            has_gimbal=has_gimbal,
            lens_type=lens_type,
            sensor_width_mm=sensor_width_mm,
            sensor_height_mm=sensor_height_mm,
            focal_length_mm=focal_length_mm,
            fixed_pitch_deg=fixed_pitch_deg
        )
