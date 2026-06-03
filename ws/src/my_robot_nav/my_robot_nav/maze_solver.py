from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 
import matplotlib.pyplot as plt

import time 


class Solver:
    @staticmethod
    def get_neigbors(maze:np.ndarray,  idx_tile: tuple[int,int])-> set[tuple[int,int]]:
        '''
        get all allowed neighbors
        '''
        shape = maze.shape
        tl_0:int = max(idx_tile[0] -1, 0)
        tl_1:int = max(idx_tile[1] -1, 0)
        br_0:int = min(idx_tile[0] +1, shape[0]-1)
        br_1:int = min(idx_tile[1] +1, shape[1]-1)

        ret:set[tuple[int,int]] = set()
        for i in range(tl_0, br_0+1):
            for j in range(tl_1, br_1+1):
                if maze[i,j] >0 : ret.add((i,j))
        ret.remove(idx_tile)
        return ret

    def figure_out_next_step(self, maze_solved:np.ndarray,  idx_tile: tuple[int,int])-> tuple[tuple[int,int], int]:
        """
        check all the vallid neigbors and return the one with the smolest val. it is assumed, that this is always better than the current one
        """
        s = self._MAXIMUM_DIST
        best_tile:tuple = None
        for n in Solver.get_neigbors(maze=maze_solved, idx_tile=idx_tile):
            if maze_solved[n] < s:
                best_tile = n
                s = maze_solved[n]
        return best_tile, s

    def __init__(self, maze_shape: tuple[int,int], goal_tile:tuple[int,int]):
        self._MAXIMUM_DIST:int = maze_shape[0] * maze_shape[1]
        self._shortest_travel_dist:int = -1
        self._goal_tile:tuple[int,int] = goal_tile
        self._maze: np.ndarray = np.ones(maze_shape)
        self._maze_solved : np.ndarray = np.ones(maze_shape)
        self._path_mask : np.ndarray = np.zeros(maze_shape,dtype=bool)
        self._radiation_diatance_map:np.ndarray = self._get_radiation_diatance_map()

    def solve(self,maze:np.ndarray, goal_tile:tuple[int,int], curr_pos_tile:tuple[int,int])->np.ndarray:
        '''
        this is similar to djkstra:
        the idea is to propergate out from the goal tile untill the current tile is reached:
        start with tiles_to_search only containing the goal, and then repeat untill the current tile is found:
            get the tile from tiles_to_search with the smalest taxi air distance
            check the 8 neighbors, if they can change their closeness, put them into tiles_to_search
        '''
        tiles_to_search: set[int]    = set([goal_tile])

        self._maze_solved[self._maze_solved > 0] = self._MAXIMUM_DIST
        self._maze_solved[goal_tile[0], goal_tile[1]] = 0

        previous_smalest_taxi_dist = self._MAXIMUM_DIST
        not_found:bool = True
        while not_found:
            tiles_to_search_next: set[int] = set()
            current_tile_to_search,previous_smalest_taxi_dist = self._get_next(tosearch_tiles=tiles_to_search, smalest_possible=previous_smalest_taxi_dist-1)
            tiles_to_search.remove(current_tile_to_search)
            curr_val = self._maze_solved[current_tile_to_search]
            next_val = curr_val + 1
            for n in Solver.get_neigbors(maze = maze, idx_tile = current_tile_to_search):
                if self._maze_solved[n] > next_val:
                    self._maze_solved[n] = next_val
                    if n == curr_pos_tile:
                        shortest_dist = self._maze_solved[n]
                        #mask = solved_maze > shortest_dist
                        #solved_maze[mask] = shortest_dist+1
                        not_found = False
                    tiles_to_search_next.add(n)
            tiles_to_search |= tiles_to_search_next


    def _update_path_mask(self, position_tile:tuple[int,int]):
        cur_pos = position_tile
        self._path_mask.fill(False)
        while cur_pos != self._goal_tile:
            self._path_mask[cur_pos] = True
            cur_pos = Solver.figure_out_next_step(self._maze_solved, cur_pos)

    def _get_radiation_diatance_map(self):
        '''
        build the taxi distance field from the goal outwards.
        generate accending x, y, with 0 at the goal, abs them, and then mesh them.
        i know i am tecnically not using the taxi metric, since i allow diagonals in the maze algorithm, but this does cut down the cost by a factor of 4
        '''
        x = np.abs(np.arange(-self._goal_tile[0],self._maze.shape[0]-self._goal_tile[0]),dtype=int)
        y = np.abs(np.arange(-self._goal_tile[1],self._maze.shape[1]-self._goal_tile[1]),dtype=int)
        x,y = np.meshgrid(y,x)
        return x+y

    def _get_next(self, tosearch_tiles: set[tuple[int,int]], smalest_possible = None)-> tuple[tuple[int,int], int]:
        '''
        search through the privided tiles, and find the best one. 
        since the algorithm aims to alsways go the best way, the previous smalest distance cant untercut by more than one.
        '''
        smalest = self._MAXIMUM_DIST
        best_tile = None
        for tile in tosearch_tiles:
            if self._radiation_diatance_map[tile] < smalest:
                best_tile = tile
                smalest = self._radiation_diatance_map[tile]
                if smalest < smalest_possible: break
        return best_tile,smalest

    @property
    def solved_maze(self)-> np.ndarray:
        return self._maze_solved
    
    @property
    def path_mask(self)-> np.ndarray:
        return self._path_mask

    



#maze=plt.imread("/Users/davidbeckschulte/Desktop/maze_100x80.png")
maze=plt.imread("/Users/davidbeckschulte/Desktop/obstacles_100x100.png")
maze = maze[:,:,0]
mask = maze < 1
maze[mask] = -1
#solved = Solver.solve()

goal = (3,3)
end = (90,71)

goal = (3,50)
end = (90,50)

#maze = Solver.solve(maze=maze, goal_tile=goal, curr_pos_tile=end)

solver = Solver(maze_shape= maze.shape, goal_tile=goal)

start = time.time()
maze_2 = solver.solve(maze= maze, goal_tile=goal, curr_pos_tile=end)
end = time.time();print(f"time {start-end}")






plt.imshow(solver.solved_maze)
plt.show()

