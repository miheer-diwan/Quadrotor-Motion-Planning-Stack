"""
main.py
    Simulates the UAV dynamics, cameras and visualizes output
    This entrypoint file runs within Blender's python interpreter

    How to setup Blender and Blender python interpreter?
        Install Blender (this code was tested on version 3.6)

        Ubuntu is officially supported. In case you are using Windows or Mac, these instructions may or may not work
              
        1. Find the python interpreter location associated with the Blender:
            a. Open Blender interactive console within Blender
            b. import sys
            c. print(sys.executable) 
        2. Use pip to install the dependencies from command prompt/terminal (I dont think it worked with powershell though). It will throw a warning and install to something called user site
            "python path" -m pip install imath numpy opencv-python scipy pyquaternion
            For example,
            Windows command looks like this,
                "C:\Program Files\Blender Foundation\Blender 3.6\3.6\python\bin\python.exe" -m pip install opencv-python ...
            Linux command looks like this in my setup,
                /usr/bin/python3.10 -m pip install opencv-python ...
        3. Reopen blender

    How to run the script?
        Create a folder called outputs and open main.blend file
        Goto scripting tab, associate it with main.py if not done already
        Run the script. It takes about 10 seconds to execute
        Goto animation tab and press space to see the visualization
    
    Based on Prof. Nitin's work https://umdausfire.github.io/teaching/fire298/asn3.html
"""

import bpy
import sys
import site

# PATH CONFIGURATION
user_site_packages =site.getusersitepackages()
sys.path.append(user_site_packages) #For pip installed dependencies
sys.path.append('./src')

# IMPORT PIP LIBS
import importlib
import math
import os
import random
import numpy as np
import cv2
import scipy
#import OpenEXR
    
# IMPORT DYNAMICS, CONTROL and USER CODE
import quad_dynamics as qd
import control
import tello
import frame_utils as frame
import rendering
import usercode
import visualization
import map
import rrt_star
import map_reader
import trajGen


# Force reload custom modules and run the latest code
importlib.reload(control)
importlib.reload(qd)
importlib.reload(tello)
importlib.reload(frame)
importlib.reload(rendering)
importlib.reload(usercode)
importlib.reload(map)
importlib.reload(rrt_star)
importlib.reload(visualization)
importlib.reload(map_reader)
importlib.reload(trajGen)

from visualization import MapVisualization, RRTVisualization, TrajVisualization
from map import Map, Cuboid
from rrt_star import RRT_star
from map_reader import MapReader
from trajGen import *

def main():
    # for debugging use print() and blender console.
    #bpy.ops.wm.console_toggle()
    
    rendering.init() 
    fps = 20
    bpy.context.scene.render.fps = fps

    animate_RRT_growth = False
    # Map 1
#    map_file = "./src/sample_maps/map1.txt"
#    start = [5,19.5,5]
#    goal = [5,-4,3]
#    (yaw, pitch, roll) = (1.57, 0, 0)
#    rrt_sample_size = 2000

    # Map 2
#    map_file = "./src/TestSetP2a/maps/map2.txt"
#    start = [0,20,2]
#    goal = [10, 20, 3]
#    (yaw, pitch, roll) = (-1.57, 0, 0)
#    rrt_sample_size = 2000

    # Map 3
    map_file = "./src/TestSetP2a/maps/map3.txt"
    start = [-1,3,2]
    goal = [20, 2, 4]
    (yaw, pitch, roll) = (0, 0, 0)
    rrt_sample_size = 3000

    # Map 4
#    map_file = "./src/sample_maps/map4.txt"
#    start = [5,16,3]
#    goal = [24,16,3]
#    (yaw, pitch, roll) = (0, 0, 0)
#    rrt_sample_size = 2000

    map_reader = MapReader(map_file)
    map = map_reader.getMap()
    map.set_buffer(0.5)
    map_visualizer = MapVisualization(map)
    map_visualizer.clear_scene()
    map_visualizer.visualize_map()


    rrt_visualizer = RRTVisualization()
    rrt_visualizer.clear_scene()
    rrt_planner = RRT_star(map)
    
    traj_visualizer = TrajVisualization()

    rrt_planner.reset_search_reslut()
    rrt_planner.set_start_and_goal(start,goal)
    rrt_planner.set_max_samples(rrt_sample_size)
    
    RRT_key_frame = 0
    if animate_RRT_growth:
        
        for i in range(rrt_sample_size):
            print(i)
            rrt_planner.search_step()
            if i <= 100:
                rrt_visualizer.visualize(rrt_planner,key_frame=RRT_key_frame)
                RRT_key_frame += 1
            if i > 100 and i <= 500:
                if i%25 == 0:
                    rrt_visualizer.visualize(rrt_planner,key_frame=RRT_key_frame,num_visible_frames=2)
                    RRT_key_frame += 2
            if i > 500 and i <= 2800:
                if i%200 == 0:
                    rrt_visualizer.visualize(rrt_planner,key_frame=RRT_key_frame,num_visible_frames=8)
                    RRT_key_frame += 8
            if i > 2800 and i <= 5000:
                if i%400 == 0:
                    rrt_visualizer.visualize(rrt_planner,key_frame=RRT_key_frame,num_visible_frames=32)
                    RRT_key_frame += 32
    else:
        rrt_planner.search()
    
    rrt_visualizer.visualize(rrt_planner,join_mesh=False, key_frame=RRT_key_frame,num_visible_frames=60)
    RRT_key_frame += 60
    bpy.ops.object.select_all(action='DESELECT')

    waypoints,_,_ = rrt_planner.retrieve_path()

    waypoints = np.array(waypoints)

    traj = trajGenerator(waypoints, max_vel = 3.0, gamma = 100)

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    # Tmax = traj.TS[-1]
    # des_state = traj.get_des_state
    trajPos = []
    
    
#    # CONSTANTS
    
    RRT_end_frame = RRT_key_frame
    sim_start_time = float(RRT_end_frame) / fps
    bpy.data.scenes['Scene'].frame_set(RRT_end_frame+1)
   # STOP time for simulation
    sim_stop_time = int(traj.TS[-1]) + 2
    
   # INIT RENDERING AND CONTROL
    # bpy.context.scene.render.fps = fps
    bpy.context.scene.frame_end = RRT_end_frame + fps*sim_stop_time
    controller = control.quad_control()
    user_sm = usercode.state_machine(traj,-yaw)

    # SET TIME STEP
    dynamics_dt = 0.01
    control_dt = controller.dt
    user_dt = user_sm.dt
    frame_dt = 1./fps

    # INIT STATES
    current_time = 0.
    xyz = np.array([start[0], -start[1], -start[2]])
    vxyz = np.array([0.0, 0.0, 0.0])
    quat = np.array([qw, qx, qy, qz])
    pqr = np.array([0.0, .0, .0])
    current_ned_state = np.concatenate((xyz, vxyz, quat, pqr))    

    # INIT TIMER
    dynamics_countdown = 0.
    control_countdown = 0.
    frame_countdown = 0.
    user_countdown = 0.

    # INIT LOG
    stateArray = current_ned_state
    timeArray = 0
    controlArray = np.array([0., 0, 0, 0])

   # SCHEDULER SUPER LOOP
   # --------------------------------------------------------------------------------------------
    while current_time < sim_stop_time:
        if frame_countdown<=0.:
            rendering.stepBlender(current_ned_state)
            frame_countdown = frame_dt

        if user_countdown<=0.:
            xyz_ned = current_ned_state[0:3]
            xyz_blender = [xyz_ned[0], -xyz_ned[1], -xyz_ned[2]]

            vxyz_ned = current_ned_state[3:6]
            vxyz_blender = [vxyz_ned[0], -vxyz_ned[1], -vxyz_ned[2]]

            xyz_bl_des, vel_bl_des, acc_bl_des, yaw_bl_setpoint = user_sm.step(current_time, xyz_blender, vxyz_blender)
            trajPos.append(xyz_bl_des.tolist())

            yaw_ned = -yaw_bl_setpoint
            WP_ned = np.array([xyz_bl_des[0], -xyz_bl_des[1], -xyz_bl_des[2], yaw_ned])
            vel_ned = np.array([vel_bl_des[0], -vel_bl_des[1], -vel_bl_des[2]])
            acc_ned = np.array([acc_bl_des[0], -acc_bl_des[1], -acc_bl_des[2]])


            user_countdown = user_dt

        if control_countdown<=0.:
            U = controller.step(current_ned_state, WP_ned, vel_ned, acc_ned)
            control_countdown = control_dt

       # Dynamics runs at base rate. 
       #   TODO replace it with ODE4 fixed step solver
        current_ned_state = current_ned_state + dynamics_dt*qd.model_derivative(current_time,
                                                            current_ned_state,
                                                            U,
                                                            tello)
       
        # UPDATE COUNTDOWNS AND CURRENT TIME
        dynamics_countdown -= dynamics_dt
        control_countdown -= dynamics_dt
        frame_countdown -= dynamics_dt
        user_countdown -=dynamics_dt
        current_time += dynamics_dt

        # LOGGING
        stateArray = np.vstack((stateArray, current_ned_state))
        controlArray = np.vstack((controlArray, U))
        timeArray = np.append(timeArray, current_time)
    # ----------------------------------------------------------------------------------------------
    user_sm.terminate()
    traj_visualizer.visualize(trajPos,user_dt,sim_start_time,material=bpy.data.materials['EndMat'],radius=0.1,resolution=3)

    # SAVE LOGGED SIGNALS TO MAT FILE FOR POST PROCESSING IN MATLAB
    loggedDict = {'time': timeArray,
                  'state': stateArray,
                  'control': controlArray}  
    scipy.io.savemat('./log/states.mat', loggedDict)
    
if __name__=="__main__":
    # donot run main.py if imported as a module
    main()
    
    
    
     