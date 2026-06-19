import math
import struct

import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def scan_to_points_in_lidar(msg: LaserScan) -> list[tuple[float, float, float]]:
    """Convert a LaserScan to (x, y, z) points in the sensor frame (z=0)."""
    pts: list[tuple[float, float, float]] = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r):
            continue
        if r <= msg.range_min or r >= msg.range_max:
            continue
        angle = msg.angle_min + i * msg.angle_increment
        pts.append((r * math.cos(angle), r * math.sin(angle), 0.0))
    return pts


def pointcloud2_from_points(points, header: Header) -> PointCloud2:
    """Pack a list of (x, y, z) tuples into a PointCloud2 message."""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = False
    buffer = bytearray()
    for x, y, z in points:
        buffer.extend(struct.pack('<fff', float(x), float(y), float(z)))
    cloud.data = bytes(buffer)
    return cloud


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


def cloud_sector_min(xy: np.ndarray, angle_min_deg: float,
                     angle_max_deg: float) -> float:
    """Minimum cartesian range over points whose bearing is in the sector.

    `xy` is an (N, 2) array of (x, y) in the robot frame. bearing = atan2(y, x)
    (0 deg = forward, + = left), range = hypot(x, y). Returns inf if no point
    falls in the sector. Cartesian analogue of `sector_min`.
    """
    if xy.shape[0] == 0:
        return float('inf')
    angles = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))
    ranges = np.hypot(xy[:, 0], xy[:, 1])
    mask = (angles >= angle_min_deg) & (angles <= angle_max_deg)
    if not mask.any():
        return float('inf')
    return float(ranges[mask].min())
