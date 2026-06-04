import variables
import numpy as np

@staticmethod
def calculate_gridsize(distance: float) -> int:
        return 2 * np.ceil(distance/variables.ROBOT_WIDTH) + 1

# @staticmethod
# def initialize_map(gridsize: int) -> np.ndarray:

class Controller:
    def __init__(self, distance):
        self._gridsize = calculate_gridsize(distance)
        # self._map = initialize_map(self._gridsize)

    def tile_to_coord(self, tile: tuple[int, int]) -> tuple[float, float]:
        x = (tile[0] - self._gridsize / 2) * variables.ROBOT_WIDTH + variables.ROBOT_WIDTH * 0.5
        y = (tile[1] - self._gridsize / 2) * variables.ROBOT_WIDTH + variables.ROBOT_WIDTH * 0.5
        return (x, y)

    def coord_to_tile(self, coordinate: tuple[float, float]) -> tuple[int, int]:
        m = (coordinate[0] - variables.ROBOT_WIDTH * 0.5) / variables.ROBOT_WIDTH + self._gridsize / 2
        n = (coordinate[0] - variables.ROBOT_WIDTH * 0.5) / variables.ROBOT_WIDTH + self._gridsize / 2
        return (m, n)
    
    # def evaluate_lidar_scan(self, map: np.ndarray)
