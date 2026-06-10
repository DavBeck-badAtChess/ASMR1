import my_robot_nav.variables as variables
import numpy as np


class Helper:
    '''
    these are the functions i use in the meta controller.
    basically a wrapper for the data in the file. all this can be static, since the data file is static by defenition
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
        tile to world.
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
        tile to world but for a single tile (ei tuple). this could have been just the same methode...
        '''
        arr = Helper.tile_to_world(np.array([[tile[0], tile[1]],[tile[0], tile[1]]]))
        return (arr[0,0], arr[0,1])

    @staticmethod
    def get_total_map_dim_in_meter() -> tuple[float, float]:
        shape = Helper.get_world_arr_shape()
        size = Helper.get_tile_size()
        return (shape[0] * size[0],shape[1] * size[1])

    @staticmethod
    def get_tiles_from_lidar_data_raw(raw_lidar_data: np.ndarray, current_coord:np.ndarray, current_heading:float) -> np.ndarray:
        '''
        build an accending angle array ranging from min to max angle (+ the current heading offset) with the same number of entries as the lidar scan,
        throw away everything that is inf or nan, use the angles to build direction vecs, multiply that with the lidar data, and convert it to tiles.
        '''
        mask = ~np.isnan(raw_lidar_data)
        mask_inf = ~np.isinf(raw_lidar_data)
        mask &= mask_inf
        dist = raw_lidar_data[mask]
        ang = np.linspace(
            variables.LIDAR_MIN_ANG + current_heading,
            variables.LIDAR_MAX_ANG + current_heading,
            raw_lidar_data.shape[0]
        )[mask]
        coords = np.column_stack([
            np.cos(ang) * dist,
            np.sin(ang) * dist
        ])
        coords[:,0] += current_coord[0]
        coords[:,1] += current_coord[1]
        return Helper.world_to_tile(coords)
