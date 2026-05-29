import math

import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def sector_min(msg: LaserScan, angle_min_deg: float, angle_max_deg: float) -> float:
    """Return minimum valid range in [angle_min_deg, angle_max_deg].

    Angles follow LaserScan convention: 0° = forward, positive = CCW (left),
    negative = CW (right). Returns inf if no valid readings exist in the sector.
    """
    angle_min_rad = math.radians(angle_min_deg)
    angle_max_rad = math.radians(angle_max_deg)
    result = float('inf')
    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        if angle_min_rad <= angle <= angle_max_rad:
            if msg.range_min < r < msg.range_max:
                result = min(result, r)
    return result


def any_below(msg: LaserScan, threshold: float) -> bool:
    """Return True if any valid range reading is below threshold."""
    return any(
        msg.range_min < r < msg.range_max and r < threshold
        for r in msg.ranges
    )


def pointcloud2_to_xy(msg: PointCloud2) -> np.ndarray:
    """Unpack a PointCloud2 to an (N, 2) array of (x, y) in the cloud's frame.

    Drops z (the LiDAR sweeps a 2D plane). Filters out NaN/inf rows. Returns
    a zero-row array if the cloud is empty.
    """
    if not msg.fields:
        return np.empty((0, 2), dtype=np.float64)
    raw = point_cloud2.read_points(
        msg, field_names=('x', 'y'), skip_nans=True
    )
    arr = np.asarray(list(raw))
    if arr.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    # read_points yields a structured dtype (named x/y fields). Unpack into a
    # plain (N, 2) float64 matrix.
    if arr.dtype.names:
        arr = np.column_stack((arr['x'], arr['y'])).astype(np.float64)
    else:
        arr = arr.reshape(-1, 2).astype(np.float64)
    finite = np.isfinite(arr).all(axis=1)
    return arr[finite]
