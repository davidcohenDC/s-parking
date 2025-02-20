import numpy as np
from typing import List, Optional

class SensorBuffer:
    """
    A helper class that manages a buffer of sensor readings
    and provides a smoothed (median) value.
    """

    def __init__(self, max_size: int) -> None:
        self.max_size: int = max_size
        self.buffer: List[float] = []

    def add(self, value: float) -> None:
        """Add a new sensor reading to the buffer."""
        self.buffer.append(value)
        if len(self.buffer) > self.max_size:
            self.buffer.pop(0)

    def median(self) -> Optional[float]:
        """Return the median of the current buffer, or None if empty."""
        if not self.buffer:
            return None
        return float(np.median(self.buffer))

    def clear(self) -> None:
        """Clear the sensor buffer."""
        self.buffer.clear()
