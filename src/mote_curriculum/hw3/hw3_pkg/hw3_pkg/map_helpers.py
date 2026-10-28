import numpy as np
import math
from scipy.ndimage import distance_transform_edt
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

MAX_LIDAR_RANGE = 12.0


def raycast(map, map_size, origin, direction, max_ittr):
    x, y = origin
    width, height = map_size
    dx, dy = direction

    for i in range(int(max_ittr)):
        x += dx
        y += dy

        # Check if the ray is out of bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            return max_ittr

        # Check if the ray hit something
        if map[int(y) * width + int(x)] == 100:
            return i

    return max_ittr


def gen_lidar_scan(particle, map: OccupancyGrid, num_rays):
    # Extract data from the particle
    x = int((particle[0] - map.info.origin.position.x) / map.info.resolution)
    y = int((particle[1] - map.info.origin.position.y) / map.info.resolution)
    theta = particle[2]
    map_data = np.array(map.data)
    map_ranges = []

    angles = np.linspace(-3.14, 3.14, num_rays)

    # Get ground truth from the map
    for angle in angles:
        map_ranges.append(
            raycast(
                map_data,
                (map.info.width, map.info.height),
                (x, y),
                (np.cos(angle + theta), np.sin(angle + theta)),
                MAX_LIDAR_RANGE / map.info.resolution,
            )
            * map.info.resolution
        )

    return map_ranges



class LikelihoodField:
    """Cached map geometry and obstacle distances, in meters."""

    def __init__(self, grid):
        self.grid = grid
        self.cells = np.asarray(grid.data, dtype=np.int16).reshape(
            grid.info.height, grid.info.width
        )
        occupied = self.cells >= 65
        self.distances = (
            distance_transform_edt(~occupied) * grid.info.resolution
            if np.any(occupied) else np.full(self.cells.shape, np.inf)
        )
        q = grid.info.origin.orientation
        self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.cos = np.cos(self.yaw)
        self.sin = np.sin(self.yaw)

    def indices(self, x, y):
        dx = np.asarray(x) - self.grid.info.origin.position.x
        dy = np.asarray(y) - self.grid.info.origin.position.y
        col = np.floor((self.cos * dx + self.sin * dy)
                       / self.grid.info.resolution).astype(int)
        row = np.floor((-self.sin * dx + self.cos * dy)
                       / self.grid.info.resolution).astype(int)
        inside = ((col >= 0) & (col < self.grid.info.width)
                  & (row >= 0) & (row < self.grid.info.height))
        return row, col, inside

    def scores(self, particles, ranges, angles, sigma, range_max):
        bearings = particles[:, 2, None] + angles[None, :]
        x = particles[:, 0, None] + ranges * np.cos(bearings)
        y = particles[:, 1, None] + ranges * np.sin(bearings)
        row, col, inside = self.indices(x, y)
        distance = np.full(x.shape, np.inf)
        distance[inside] = self.distances[row[inside], col[inside]]
        # Unknown cells provide no evidence of a mapped obstacle.
        known = np.zeros(x.shape, dtype=bool)
        known[inside] = self.cells[row[inside], col[inside]] >= 0
        distance[~known] = np.inf
        beam = 0.5 * np.exp(-0.5 * (distance / sigma) ** 2) + 0.5 / range_max
        scores = 1.0 + np.sum(beam ** 3, axis=1)
        row, col, inside = self.indices(particles[:, 0], particles[:, 1])
        free = np.zeros(len(particles), dtype=bool)
        free[inside] = self.cells[row[inside], col[inside]] == 0
        scores[~free] = 0.0
        return scores

    def scan(self, particle, angles, range_max=MAX_LIDAR_RANGE):
        """Predict the ranges for one lidar pose."""
        return self.scans(np.asarray(particle)[None, :], angles, range_max)[0]

    def scans(self, particles, angles, range_max=MAX_LIDAR_RANGE):
        """March rays for all particles together using cached map cells."""
        particles = np.asarray(particles)
        angles = np.asarray(angles)
        bearings = particles[:, 2, None] + angles[None, :]
        dx, dy = np.cos(bearings).ravel(), np.sin(bearings).ravel()
        origin_x = np.repeat(particles[:, 0], len(angles))
        origin_y = np.repeat(particles[:, 1], len(angles))
        result = np.full(bearings.size, range_max, dtype=float)
        active = np.ones(bearings.size, dtype=bool)
        for distance in np.arange(0.0, range_max, self.grid.info.resolution):
            ids = np.flatnonzero(active)
            if not len(ids):
                break
            row, col, inside = self.indices(
                origin_x[ids] + distance * dx[ids],
                origin_y[ids] + distance * dy[ids],
            )
            active[ids[~inside]] = False
            hits = np.zeros(len(ids), dtype=bool)
            hits[inside] = self.cells[row[inside], col[inside]] >= 65
            result[ids[hits]] = distance
            active[ids[hits]] = False
        return result.reshape(bearings.shape)
