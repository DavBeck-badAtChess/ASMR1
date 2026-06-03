from __future__ import annotations
import sys
sys.dont_write_bytecode = True
import numpy as np 






class Solver:
    LARGEST:int = 1000 # TODO this should be calculated with the matrix size

    @staticmethod
    def get_neigbors(maze:np.ndarray,  idx_tile: tuple[int,int])-> set[tuple[int,int]]:
        '''
        get all allowed neighbors
        '''
        shape = maze.shape
        tl_0:int = max(idx_tile[0] -1, 0)
        tl_1:int = max(idx_tile[1] -1, 0)
        br_0:int = min(idx_tile[0] +1, shape[0])
        br_1:int = min(idx_tile[1] +1, shape[1])

        ret:set[tuple[int,int]] = set()
        for i in range(tl_0, br_0+1):
            for j in range(tl_1, br_1+1):
                if maze[i,j] >0 : ret.add((i,j))
        ret.remove(idx_tile)
        return ret

    @staticmethod
    def figure_out_next_step(maze_solved:np.ndarray,  idx_tile: tuple[int,int])-> tuple[tuple[int,int], int]:
        """
        check all the vallid neigbors and return the one with the smolest val. it is assumed, that this is always better than the current one
        """
        s = Solver.LARGEST
        best_tile:tuple = None
        for n in Solver.get_neigbors(maze=maze_solved, idx_tile=idx_tile):
            if maze_solved[n] < s:
                best_tile = n
                s = maze_solved[n]
        return best_tile, s

    @staticmethod
    def solve(maze:np.ndarray, goal_tile:tuple[int,int], curr_pos_tile:tuple[int,int])->np.ndarray:
        '''
        it works like this (innefficient for now):
        it starts witht he goal_tile, sees the surrounding tiles, sets all of them to:

            min(value so faar, value of tile +1)
        
        if the tile has changed, it is stored in the buffer,these are searched in the next iter.
        (here it could be more efficient, it is smarter to search some idx than others).
        the whole thing terminates, when the goal is found (this is guarantueed to be the shortest path)
        '''
        to_search:   set[int] = set([goal_tile])
        solved_maze: np.ndarray = maze.copy()

        solved_maze_nothing_mask = solved_maze > 0 
        solved_maze[solved_maze_nothing_mask]= Solver.LARGEST
        solved_maze[goal_tile[0], goal_tile[1]] = 0

        shortest_dist:int
        found:bool = False
        while not found:
            tos_next: set[int] = set()
            for tos in to_search:
                curr_val = solved_maze[tos]
                next_val = curr_val + 1
                for n in Solver.get_neigbors(maze =maze, idx_tile= tos):
                    print(solved_maze[n])
                    if solved_maze[n] > next_val:
                        solved_maze[n] = next_val
                        if n == curr_pos_tile:
                            shortest_dist = solved_maze[n]
                            mask = solved_maze > shortest_dist
                            solved_maze[mask] = shortest_dist+1
                            found = True
                        tos_next.add(n)
            to_search = tos_next
        return solved_maze


    def __init__(self):
        pass

    


