import my_robot_nav.variables as variables
import variables as variables
#import numpy as np






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
        i = np.round(coord_array[:,0] / Helper.get_tile_size()[0]) + s[0] // 2
        j = np.round(coord_array[:,1] / Helper.get_tile_size()[1]) + s[1] // 2
        #print("i shape", i.shape)
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
        tile_arr_c = tile_arr.copy().astype(float)
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
        arr = Helper.tile_to_world(np.array([[tile[0], tile[1]],[tile[0], tile[1]]]))
        return (arr[0,0], arr[0,1])

    @staticmethod
    def get_total_map_dim_in_meter() -> tuple[float, float]:
        shape = Helper.get_world_arr_shape()
        size = Helper.get_tile_size()
        return (shape[0] * size[0],shape[1] * size[1])

    @staticmethod
    def get_tiles_from_lidar_data_raw(raw_lidar_data: np.ndarray) -> np.ndarray:
        mask = ~np.isnan(raw_lidar_data)
        dist = raw_lidar_data[mask]
        ang = np.linspace(
            variables.LIDAR_MIN_ANG,
            variables.LIDAR_MAX_ANG,
            raw_lidar_data.shape[0]
        )[mask]
        coords = np.column_stack([
            np.cos(ang) * dist,
            np.sin(ang) * dist
        ])
        return Helper.world_to_tile(coords)

    @staticmethod
    def get_starting_tile()->tuple[int,int]:
        '''
        return the tile at the very beginning
        '''
        return Helper.world_to_tile_single(variables.START_COORDS)
    

# test_scan = np.linspace(0,4, 36)
# t = Helper.get_tiles_from_lidar_data_raw(test_scan)

# print("shape", Helper.get_world_arr_shape())
# print("tile_size", Helper.get_tile_size())

# print("max tile", np.max(t))
# print(np.max(t))
""" 
print(Helper.get_tile_size())
test = np.linspace(0,3,360)
print(Helper.get_tiles_from_lidar_data_raw(test))
 """