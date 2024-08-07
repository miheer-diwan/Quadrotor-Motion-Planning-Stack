# from visualization import MapVisualization, RRTVisualization
from map import Map, Cuboid
from rrt_star import RRT_star
from map_reader import MapReader
from trajGen import *
from trajutils import *
import numpy as np

map_file = "/home/blacksnow/drones/YourDirectoryID_p2a/src/sample_maps/map4.txt"  # Replace with the path to your map file
map_reader = MapReader(map_file)
map = map_reader.getMap()
# map_visualizer = MapVisualization(map)
# rrt_visualizer = RRTVisualization()
rrt_planner = RRT_star(map)

start = [5,16,3]
goal = [24,16,3]

rrt_planner.reset_search_reslut()
rrt_planner.set_start_and_goal(start,goal)
rrt_planner.set_max_samples(2000)
rrt_planner.search()
waypoints,_,_ = rrt_planner.retrieve_path()

waypoints = np.array(waypoints)

traj = trajGenerator(waypoints, max_vel = 5, gamma = 1e6)

Tmax = traj.TS[-1]
des_state = traj.get_des_state
