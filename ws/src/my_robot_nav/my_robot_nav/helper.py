import my_robot_nav.variables as variables
#import variables as variables
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

    # # TODO: Implement lidar evaluation with proper lidar message by changing lidar_scan to type msg and extracting values with scan_values = np.array(msg.ranges)
    # def evaluate_lidar_scan(self, lidar_scan: msg, current_pos: tuple[float, float]):
    #     '''
    #     Uses lidar's message as well as robot's current position to calculate the position of an obstacle and map it.
    #     '''
    #     scan_values = np.array(msg.ranges)
    #     angles = np.arrange(len(scan_values)) * (360 / len(scan_values)) / 180 * np.pi
    #     result = np.column_stack((scan_values, angles))
    #     result = result[~np.isnan(result).any(axis=0)]
    #     obstacle_pos = np.column_stack((np.cos(result[:, 1]) * result[:, 0] + current_pos[0], np.sin(result[:, 1]) * result[:, 0] + current_pos[1]))
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
        return (variables.ROBOT_WIDTH,variables.ROBOT_WIDTH)

    @staticmethod
    def get_world_arr_shape()->tuple[int, int]:
        '''
        return the shape of the world arr
        '''
        n = int((variables.MAX_DISTANCE // variables.ROBOT_WIDTH) * 2 + 1)
        return (n,n)

    @staticmethod
    def world_to_tile(coord_array: np.ndarray) -> np.ndarray:
        '''
        take an array of xy coords, and returns a mask
        '''
        s = Helper.get_world_arr_shape()
        i = np.round(coord_array[0] / variables.ROBOT_WIDTH) + s[0] // 2
        j = np.round(coord_array[1] / variables.ROBOT_WIDTH) + s[1] // 2
        return np.stack([i,j], axis=1).astype(int)

    @staticmethod
    def world_to_tile_single(single_coord: np.ndarray) -> tuple[int,int]:
        '''
        take a sigle xy point ( arr of shapw (2,)) and return the corresponding tile 
        '''
        s = Helper.get_world_arr_shape()
        i = int(np.round(single_coord[0] / variables.ROBOT_WIDTH) + s[0] // 2)
        j = int(np.round(single_coord[1] / variables.ROBOT_WIDTH) + s[1] // 2)
        return (i,j)

    @staticmethod
    def tile_to_world(tile_arr: np.ndarray) -> np.ndarray:
        '''
        bla bla bla
        '''
        tile_arr_c = tile_arr.copy()
        shape = Helper.get_world_arr_shape()
        size = Helper.get_tile_size()
        tile_arr_c[:,0] -= shape[0]//2 -size[0]/2 # to position them at the center. not really needet though...
        tile_arr_c[:,1] -= shape[1]//2 -size[1]/2
        tile_arr_c[:,0] *= size[0]
        tile_arr_c[:,1] *= size[1]
        return tile_arr_c

    @staticmethod
    def tile_to_world_single(tile:tuple[int,int]) -> np.ndarray:
        '''
        bla bla
        '''
        arr = Helper.tile_to_world(np.array([tile[0], tile[1]]))
        return (arr[0], arr[1])

    #@staticmethod
    #def get_goal_tile()-> tuple[int,int]:
    #    return Helper.world_to_tile_single(variables.GOAL_COORDS)

    @staticmethod
    def get_total_map_dim_in_meter() -> tuple[float, float]:
        shape = Helper.get_world_arr_shape()
        size = Helper.get_tile_size()
        return (shape[0] * size[0],shape[1] * size[1])

    @staticmethod
    def get_tiles_from_lidar_data_raw(raw_lidar_data:np.ndarray)-> np.ndarray:
        '''
        returns a mask of the seen obstacles, obstacle -> True, else False
        first match the messurements to the angles, then throw out nan, then generate the dir vecs, ten mult with the messurements. 
        '''
        ang = np.linspace(start=variables.LIDAR_MIN_ANG, stop=variables.LIDAR_MAX_ANG, num= raw_lidar_data.shape[0])
        compund = np.stack([ang, raw_lidar_data] , axis= 1)
        compund = compund[~np.isnan(compund[:,1])]
        dir_x = np.cos(compund[:, 0])
        dir_y = np.sin(compund[:, 0])
        coords = np.stack([dir_x, dir_y], axis=1)
        coords[:,0] *= compund[:,0]
        coords[:,1] *= compund[:,0]
        return Helper.world_to_tile(coords)

    @staticmethod
    def get_starting_tile()->tuple[int,int]:
        '''
        return the tile at the very beginning
        '''
        return Helper.world_to_tile_single(variables.START_COORDS)
    


# test = np.linspace(0,36,36)
# Helper.get_mask_from_lidar_data_raw(test)
