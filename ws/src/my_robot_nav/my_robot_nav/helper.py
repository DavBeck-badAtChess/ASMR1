import variables
import numpy as np

@staticmethod
def calculate_gridsize(distance: float) -> int:
        return 2 * np.ceil(distance/variables.ROBOT_WIDTH) + 1

class Controller:
    def __init__(self, distance):
        self._gridsize = calculate_gridsize(distance)
        self._map: np.ndarray = np.ones((self.gridsize, self.gridsize))

    def tile_to_coord(self, tile: tuple[int, int]) -> tuple[float, float]:
        x = (tile[0] - self._gridsize / 2) * variables.ROBOT_WIDTH + variables.ROBOT_WIDTH * 0.5
        y = (tile[1] - self._gridsize / 2) * variables.ROBOT_WIDTH + variables.ROBOT_WIDTH * 0.5
        return (x, y)

    def coord_to_tile(self, coordinate: tuple[float, float]) -> tuple[int, int]:
        m = np.round(coordinate[0] / variables.ROBOT_WIDTH) + self._gridsize // 2
        n = np.round(coordinate[1] / variables.ROBOT_WIDTH) + self._gridsize // 2
        return (m, n)
    
    def coords_to_tiles(self, coords: np.ndarray) -> np.ndarray:
        return np.round(
            coords / variables.ROBOT_WIDTH
        ).astype(int) + self._gridsize // 2

    # TODO: Implement lidar evaluation with proper lidar message by changing lidar_scan to type msg and extracting values with scan_values = np.array(msg.ranges)
    def evaluate_lidar_scan(self, lidar_scan: msg, current_pos: tuple[float, float]):
        '''
        Uses lidar's message as well as robot's current position to calculate the position of an obstacle and map it.
        '''
        scan_values = np.array(msg.ranges)
        angles = np.arrange(len(scan_values)) * (360 / len(scan_values)) / 180 * np.pi
        result = np.column_stack((scan_values, angles))
        result = result[~np.isnan(result).any(axis=0)]
        obstacle_pos = np.column_stack((np.cos(result[:, 1]) * result[:, 0] + current_pos[0], np.sin(result[:, 1]) * result[:, 0] + current_pos[1]))
        obstacle_tiles = coords_to_tile(self, obstacle_pos)

        # index = 0
        # for value in scan_values:
        #     if value != nan:   
        #         angle = (360 / len(scan_values) * index) / 180 * np.pi
        #         x = np.cos(angle) * value
        #         y = np.sin(angle) * value
        #         obstacle_pos = (current_pos[0] + x, current_pos[0] + y)
        #         obstacle_tile = coord_to_tile(self, obstacle_pos)
        #         self._map[obstacle_tile] = -1
        #     index += 1



