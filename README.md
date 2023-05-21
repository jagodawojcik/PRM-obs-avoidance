# PRM-obs-avoidance
Python implementation Probabilistic Roadmap Planner for robotic path planning and obstacle avoidance in 2d grid environment.


#The PRM algorithm steps
1. generate a number of sample points uniformly at random, check each point for collision, leave samples in free config space only. The number of generated points is specified by N_NODES parameter;
2. Add start and end points to the graph;
3. Connect each point with the nodes in their neighbourhood specified by RADIUS parameter;
4. Run Dijkstra's to find the shortest path in the created graph.


Object of class PRM requires to specify:
- start and end position of the robot as (x,y) coordinates;
- obstacle list
- 2d map size (x,y), produces a square grid
- robot radius

The obstacles are defined as a list of (x, y, r) coordinates, where 'x','y' define the position of the center of the circular obstacle in the 2d grid, while the parameter 'r' is the radius of the obstacle.

##Solution Example Plots




