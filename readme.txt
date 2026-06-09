


the main idea here is to reconstruct the world in a grid using the lidar data,
apply a pathfinding algorithm to find the shortest path to the goal, and use a point navigator to navigate to the next point on the 
generated path.

maze solving:
    the maze is solved using a floodfill approach (i think this is just djkstra).
    the maze consists of 3 layers:
    - maze      : this is just the geometry data gathered by the lidar
    - soft maze : this is the raw lidar data + some buffering on the walls, to enforce a minimal distance from actual walls
    - solved soft maze : this is the soft maze + the distance to the goal tile allong an unobstructed path.
    - (also a path mask)
    the maze and softmaze are updated when the lidar reads anything new.
    the solved maze is only updated if the softmaze intersects with the current pash.

the used debug tool is a "map", which displays data such as:
    - the current path
    - the current tile the robot occupies
    - the distances of the maze

problems:
    - the maze solving should really happen in a different thread
    - the paths are alwas aa or diagonal. this is not the shortest path in a space with the eukl metric.
    - lidar/odom asincronicity can cause the robot to hallucinate walls that are not there. 
        in bad cases this can lead the robot to thinking that there is no path to the goal
    
strenghts:
    - the robot can solve mazes, since the knowledge about the world is not just one lidar scan.