from typing import Optional


def combine_lidar_distances(
    front_min: Optional[float],
    target_dist: Optional[float],
) -> Optional[float]:
    """Use the closer of front-sector min and target-column range."""
    values = [v for v in (front_min, target_dist) if v is not None]
    return min(values) if values else None
