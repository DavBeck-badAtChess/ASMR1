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
#        obstacle_tiles = coords_to_tile(self, obstacle_pos)

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




class Helper:
    '''
    basically a wrapper for the data in the file. all this can be static, since the data file is static by defenition
    these are the functions i would like to have 
    '''
    @staticmethod
    def get_tile_size()-> tuple[int,int]:
        '''
        returns the x,y size of a sinlge tile (i think the was the robot size)
        '''
        pass

    @staticmethod
    def get_world_arr_shape()->tuple[int, int]:
        '''
        return the shape of the world arr
        '''
        pass

    @staticmethod
    def world_to_tile(coord_array: np.ndarray) -> np.ndarray:
        '''
        take an array of xy coords, and returns a mask
        '''
        pass

    @staticmethod
    def world_to_tile_single(single_coord: np.ndarray) -> tuple[int,int]:
        '''
        take a sigle xy point ( arr of shapw (2,)) and return the corresponding tile 
        '''
        pass

    @staticmethod
    def tile_to_world_single(tile:tuple[int,int]) -> np.ndarray:
        '''
        bla bla
        '''
        pass

    @staticmethod
    def tile_to_world(tile_arr: np.ndarray) -> np.ndarray:
        '''
        bla bla bla
        '''
        pass

    @staticmethod
    def get_goal_tile()-> tuple[int,int]:
        pass

    @staticmethod
    def get_total_map_dim_in_meter() -> tuple[float, float]:
        pass

    @staticmethod
    def get_mask_from_lidar_data_raw(raw_lidar_data:np.ndarray)-> np.ndarray:
        '''
        returns a mask of the seen obstacles, obstacle -> True, else False
        
        '''

    @staticmethod
    def get_starting_tile()->tuple[int,int]:
        '''
        return the tile at the very beginning
        '''
        pass
