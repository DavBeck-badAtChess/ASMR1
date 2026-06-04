from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
from enum import Enum

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
        get all allowed neighbors
        '''
        shape = maze.shape
        tl_0:int = max(idx_tile[0] -1, 0)
        tl_1:int = max(idx_tile[1] -1, 0)
        br_0:int = min(idx_tile[0] +2, shape[0])
        br_1:int = min(idx_tile[1] +2, shape[1])

        ret:set[tuple[int,int]] = set()
        for i in range(tl_0, br_0):
            for j in range(tl_1, br_1):
                if maze[i,j] >0 : ret.add((i,j))
        ret.remove(idx_tile)
        return ret

    def __init__(self, maze_shape: tuple[int,int], goal_tile:tuple[int,int]):
        self._MAXIMUM_DIST:int = maze_shape[0] * maze_shape[1]
        self._shortest_travel_dist:int = -1
        self._goal_tile:tuple[int,int] = goal_tile
        self._maze: np.ndarray = np.ones(maze_shape)
        self._maze_solved : np.ndarray = np.ones(maze_shape)
        self._path_mask : np.ndarray = np.zeros(maze_shape, dtype=bool)
        self._dirty_surrounding_flag:bool = True

    def get_next_direction(self, tile_position:tuple[int,int]) -> set[DIRECTION]:
        '''
        check if the maps (paths etc) need to be updated, and returns the DIRECTION compund of the next step
        '''
        if self._dirty_surrounding_flag:
            self._solve_maze(position_tile=tile_position)
            self._update_path_mask(position_tile=tile_position)
        target_tile = self._figure_out_next_step(tile_position=tile_position)
        return DIRECTION.compund(target_tile = target_tile, tile_source=tile_position)

    def account_for_geometry(self, new_geometry_mask:np.ndarray):
        '''
        i assume a correctly sized mask containing all the new geometry as true, rest false
        '''
        self._maze[new_geometry_mask] = -1
        if np.any(new_geometry_mask and self._path_mask):
            self._dirty_surrounding_flag = True

    def _figure_out_next_step(self, position_tile: tuple[int,int])-> tuple[tuple[int,int], int]:
        """
        check all the vallid neigbors and return the one with the smolest val. it is assumed, that this is always better than the current one
        """
        s = self._MAXIMUM_DIST
        best_tile:tuple = None
        for n in Solver.get_neigbors(maze=self._maze_solved, idx_tile=position_tile):
            if self._maze_solved[n] < s:
                best_tile = n
                s = self._maze_solved[n]
        return best_tile

    def _solve_maze(self, position_tile:tuple[int,int])->np.ndarray:
        '''
        it works like this (innefficient for now):
        it starts witht he goal_tile, sees the surrounding tiles, sets all of them to:

            min(value so faar, value of tile +1)
        
        if the tile has changed, it is stored in the buffer,these are searched in the next iter.
        (here it could be more efficient, it is smarter to search some idx than others).
        the whole thing terminates, when the goal is found (this is guarantueed to be the shortest path)
        '''
        tiles_to_search:   set[int] = set([self._goal_tile])

        self._maze_solved[self._maze_solved > 0] = self._MAXIMUM_DIST
        self._maze_solved[self._goal_tile[0], self._goal_tile[1]] = 1

        found:bool = False
        while not found:
            tiles_to_search_next: set[int] = set()
            for tile in tiles_to_search:
                curr_val = self._maze_solved[tile]
                next_val = curr_val + 1
                for n in Solver.get_neigbors(maze = maze, idx_tile = tile):
                    if self._maze_solved[n] > next_val:
                        self._maze_solved[n] = next_val
                        if n == position_tile:
                            shortest_dist = self._maze_solved[n]
                            mask = self._maze_solved > shortest_dist
                            self._maze_solved[mask] = shortest_dist+1
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
    def solved_maze(self)-> np.ndarray:
        return self._maze_solved
    
    @property
    def path_mask(self)-> np.ndarray:
        return self._path_mask

