from typing import Dict, Any


class ParkingConfig:
    """Encapsulates configuration parameters for the parking controller."""

    def __init__(self, config_dict: Dict[str, Any]):
        self.config = config_dict

    def get(self, key: str, default: Any = None) -> Any:
        """Retrieves a configuration value, providing a default if not found."""
        return self.config.get(key, default)