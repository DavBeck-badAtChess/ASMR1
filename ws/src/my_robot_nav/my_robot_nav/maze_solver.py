from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum

#import matplotlib.pyplot as plt
#from helper import Helper

'''
TODO 
the maze should really be a bool map. 
there needs to be an intermediate maze map, with buffer zones arround the actual obsticals (bool map too).
other than that, this should be fine
'''

class DIRECTION(Enum):
    UP =   (0,np.array([0,1]))
    DOWN = (1,np.array([0,-1]))
    RIGHT =  (2, np.array([1.0]))
    LEFT = (3,np.array([-1,0]))

    def __init__(self, id:int, dir:np.ndarray):
        super().__init__(id)
        self._vec_dir:np.ndarray = dir

    @property
    def vec_dir(self)->np.ndarray:
        return self._vec_dir

    @staticmethod
    def resolve_compund_to_vec_dir(compund:set[DIRECTION])->np.ndarray:
        ret :np.ndarray = np.zeros((2,))
        for en in compund:
            ret += en.vec_dir
        ret /= np.linalg.norm(ret)
        return ret

    @staticmethod
    def compund(tile_source:tuple[int,int], tile_target:tuple[int,int])->set[DIRECTION]:
        compund:set[DIRECTION] = set()
        if tile_source[0] < tile_target[0]:
            compund.add(DIRECTION.RIGHT)
        elif tile_source[0] > tile_target[0]:
            compund.add(DIRECTION.LEFT)
        if tile_source[1] < tile_target[1]:
            compund.add(DIRECTION.UP)
        elif tile_source[1] > tile_target[1]:
            compund.add(DIRECTION.DOWN)
        return compund


class Solver:
    @staticmethod
    def get_neigbors(maze:np.ndarray,  idx_tile: tuple[int,int])-> set[tuple[int,int]]:
        '''
        get all allowed neighbors (this is not exclusively used by boolean arrs)
        '''
        shape = maze.shape
        tl_0:int = max(idx_tile[0] -1, 0)
        tl_1:int = max(idx_tile[1] -1, 0)
        br_0:int = min(idx_tile[0] +2, shape[0])
        br_1:int = min(idx_tile[1] +2, shape[1])

        ret:set[tuple[int,int]] = set()
        for i in range(tl_0, br_0):
            for j in range(tl_1, br_1):
                if maze[i,j] > 0 and (i,j) != idx_tile: ret.add((i,j))
        return ret

    def __init__(self, maze_shape: tuple[int,int], goal_tile:tuple[int,int]):
        '''
        have three levels top down stream:
            - _maze:        this stores the actual seen data in a bool arr. 
            - _maze_soft:   this is basically the orriginal maze, but it has a buffer arround the lidar data, so the robot does not drive directly next to occupied tiles 
            - _maze_soft_solved: this is a solved version of _maze_soft, it has -1 for undrivable tiles, and the reset is a heatmap of of the (navigatable) distance to the gaol

            - _path_mask:   once tha maze is solved, a path is generated to be taken. this path is stored as a mask. if new lidar data comes in, it is checked, wheter the soft 
                            data obstructs the path, if not, save it, but dont regen the path. else also regen the path.
        '''
        self._MAXIMUM_DIST:int = maze_shape[0] * maze_shape[1]

        self._goal_tile:tuple[int,int] = goal_tile
        self._maze: np.ndarray = np.ones(maze_shape, dtype=bool)
        self._maze_soft: np.ndarray = np.ones(maze_shape, dtype=bool)
        self._maze_soft_solved : np.ndarray = np.ones(maze_shape)
        self._path_mask : np.ndarray = np.zeros(maze_shape, dtype=bool)
        self._dirty_surrounding_flag:bool = True

    def get_next_direction(self, tile_position:tuple[int,int]) -> set[DIRECTION]:
        '''
        check if the maps (paths etc) need to be updated, and returns the DIRECTION compund of the next step
        '''
        if self._dirty_surrounding_flag:
            self._solve_maze(position_tile=tile_position)
            self._update_path_mask(position_tile=tile_position)
        target_tile = self._figure_out_next_step(position_tile=tile_position)
        return DIRECTION.compund(tile_target = target_tile, tile_source=tile_position)
    
    def get_next_tile(self, tile_position:tuple[int,int]) -> tuple[int,int]:
        '''
        check if the maps (paths etc) need to be updated, and returns the DIRECTION compund of the next step
        '''
        if self._dirty_surrounding_flag:
            self._solve_maze(position_tile=tile_position)
            self._update_path_mask(position_tile=tile_position)
        target_tile = self._figure_out_next_step(position_tile=tile_position)
        return target_tile

    def account_for_geometry(self, new_geometry_tiles:np.ndarray):
        '''
        i assume a correctly sized mask containing all the new geometry as true, rest false.

        if the data was not seen before, put it in, recalc the soft map, see if the path is now 
        obstructed and set the flag accordingly
        '''
        new_information:bool = False
        print(new_geometry_tiles.shape)
        print(np.max(new_geometry_tiles))
        print(self._maze.shape)
        new_geometry_mask = np.zeros(self._maze.shape,dtype=bool)
        new_geometry_mask[
            new_geometry_tiles[:, 0],
            new_geometry_tiles[:, 1]
        ] = True
        if np.all(new_geometry_mask | self._maze):# see if new_geometry_mask -> self._maze
            print("NEW"*10)
            #self._maze[~new_geometry_mask] = False
            self._maze[
                new_geometry_tiles[:, 0],
                new_geometry_tiles[:, 1]
            ] = False
            self._update_soft_maze()
            new_information = True
        if np.any(self._maze_soft & self._path_mask):
            self._dirty_surrounding_flag = True
        return new_information


    def _figure_out_next_step(self, position_tile: tuple[int,int])-> tuple[tuple[int,int], int]:
        """
        check all the vallid neigbors and return the one with the smolest val. it is assumed, that this is always better than the current one
        """
        s = self._MAXIMUM_DIST
        best_tile:tuple = None
        for n in Solver.get_neigbors(maze=self._maze_soft_solved, idx_tile=position_tile):
            if self._maze_soft_solved[n] < s:
                best_tile = n
                s = self._maze_soft_solved[n]
        return best_tile
    
    def _update_soft_maze(self):
        '''
        assume that all the sensor data is in the maze, and derrive the smooth maze from scratch.
        i just roll it in every direction and add it. the clean version here is to use a gaussian filter and clamp it (esp cause
        this would be invariant against resoloution), 
        ussually scipy hase you covered here, but i dont think we have acces to it in this env
        '''
        self._maze_soft = self._maze & self._maze_soft
        self._maze_soft = np.roll(a = self._maze, shift=1, axis=0) & self._maze_soft
        self._maze_soft = np.roll(a = self._maze, shift=-1, axis=0) & self._maze_soft
        self._maze_soft = np.roll(a = self._maze, shift=1, axis=1) & self._maze_soft
        self._maze_soft = np.roll(a = self._maze, shift=-1, axis=1) & self._maze_soft

    def _solve_maze(self, position_tile:tuple[int,int])->np.ndarray:
        '''
        it works like this (innefficient for now):
        it starts witht he goal_tile, sees the surrounding tiles, sets all of them to:

            min(value so faar, value of tile +1)
        
        if the tile has changed, it is stored in the buffer,these are searched in the next iter.
        (here it could be more efficient, it is smarter to search some idx than others, but my impl of that was actually slower, so ill stick with this).
        the whole thing terminates, when the goal is found (this is guarantueed to be the shortest path)
        '''
        tiles_to_search:   set[int] = set([self._goal_tile])
        self._maze_soft_solved[self._maze_soft_solved > 0] = self._MAXIMUM_DIST
        self._maze_soft_solved[self._goal_tile[0], self._goal_tile[1]] = 1
        found:bool = False
        while not found:
            tiles_to_search_next: set[int] = set()
            for tile in tiles_to_search:
                curr_val = self._maze_soft_solved[tile]
                next_val = curr_val + 1
                for n in Solver.get_neigbors(maze = self._maze_soft, idx_tile = tile):
                    if self._maze_soft_solved[n] > next_val:
                        self._maze_soft_solved[n] = next_val
                        if n == position_tile:
                            shortest_dist = self._maze_soft_solved[n]
                            mask = self._maze_soft_solved > shortest_dist
                            self._maze_soft_solved[mask] = shortest_dist+1
                            found = True
                        tiles_to_search_next.add(n)
            tiles_to_search = tiles_to_search_next

    def _update_path_mask(self, position_tile:tuple[int,int]):
        '''
        walk along the ideal path and save it in the boolean mask
        '''
        cur_pos = position_tile
        self._path_mask.fill(False)
        while cur_pos != self._goal_tile:
            self._path_mask[cur_pos] = True
            cur_pos = self._figure_out_next_step(cur_pos)

    @property
    def informational_map(self)->np.ndarray:
        '''
        return an array containing all the interesting data.
        '''
        inf = np.ones(self._maze.shape)
        inf[self._maze] += 1
        inf[self._maze_soft] += 1
        inf[self._path_mask] = -1
        return inf

    @property
    def solved_maze(self)-> np.ndarray:
        return self._maze_soft_solved
    
    @property
    def path_mask(self)-> np.ndarray:
        return self._path_mask




# test = np.zeros((200,200), dtype=bool)
# scan = np.linspace(0,2, 36)
# solver = Solver(test.shape, (10,10))
# scan_idx = Helper.get_tiles_from_lidar_data_raw(scan)
# print("scan idx", scan_idx.shape)
# solver.account_for_geometry(scan_idx)
# solver._update_soft_maze()
# plt.imshow(solver._maze_soft)
# plt.show()


""" test_maze = plt.imread("/Users/davidbeckschulte/Desktop/obstacles_100x100.png")[:,:,0]
test_maze = (test_maze-test_maze.min())/(test_maze.max()-test_maze.min())
print(test_maze)
test_maze = test_maze>0.5 





solver = Solver(test_maze.shape, goal_tile=(5,5))

print(test_maze.shape)
solver.account_for_geometry(test_maze)

solver.get_next_direction((90, 71))
t = solver._maze_soft.copy().astype(int)
t[solver._maze] -=1
plt.imshow(solver.informational_map)
plt.show()
 """