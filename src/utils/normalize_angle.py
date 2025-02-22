def normalize_angle(angle: float) -> float:
    """Normalize angle to the range [-180, 180)."""
    while angle < -180:
        angle += 360
    while angle >= 180:
        angle -= 360
    return angle