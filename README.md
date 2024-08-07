# Quadrotor Motion Planning Stack

## Project Overview
In this project, I implemented path and trajectory planning (motion planning) and tuned a control stack for a quadrotor to navigate from a start position to a goal position through a pre-mapped or known 3D environment. This project is divided into multiple parts, including environment setup, path planning, trajectory generation, and controller tuning. The initial implementation will be done in simulation using Blender. 

Please refer to the [Report](Report.pdf) for more detailed information.

## Environment Setup
The simulation environment is based on Blender. The map and the start and goal positions are loaded dynamically as described in a text file. An example environment file format is given below:

```
boundary xmin ymin zmin xmax ymax zmax
block xmin ymin zmin xmax ymax zmax r g b
boundary 0 0 0 45 35 6
block 1.0 1.0 1.0 3.0 3.0 3.0 0 255 0
block 20.0 10.0 0.0 21.0 20.0 6.0 0 0 255
```
I wrote a function to read and display this map in Blender. The start and goal positions are shown as unit red and green spheres respectively. Obstacles are bloated to account for the quadrotor size.

## Path Planner
I implemented an RRT* global path planner function that takes the start position, goal position, and map path as inputs. The planner visualizes exploration paths and nodes in Blender using cylinders and spheres.

## Trajectory Generation
I converted the waypoints obtained from the RRT* planner into a cubic/quintic spline trajectory. I ensured that the generated trajectory is smooth and dynamically feasible. I implemented collision checking to validate paths.

## Controller
I tuned a cascaded PID controller to follow the desired trajectory. The controller ensures that the quadrotor follows the trajectory as closely as possible, minimizing overshoots and avoiding collisions.

## Key Components
1. **Map Reader and Display**: Function to read and display the environment map in Blender.
2. **RRT* Path Planner**: Implemented to generate a path from the start to the goal position.
3. **Trajectory Planner**: Converts the path into a smooth and feasible trajectory.
4. **PID Controller**: Tuned to follow the generated trajectory, ensuring minimal overshoot and collision avoidance.

## Results
I visualized the environment, paths, and trajectories in Blender. I plotted the desired and actual positions, velocities, and 3D trajectories to validate the controller's performance.

### Generated Map with Obstacles:
![map1_ortho_buff](https://github.com/user-attachments/assets/8b3c44c5-eb61-472b-886b-b5f928edc3ff)

### RRT* algorithm in action:
![Screenshot from 2023-10-02 22-42-40](https://github.com/user-attachments/assets/c884fab3-7e06-40f1-87a8-81700b033f31)

### Highlighted Best Path:
![rrt_path_ortho](https://github.com/user-attachments/assets/188fbc98-3bf9-4fbb-9bd1-e298d90e7daa)

### Video:


https://github.com/user-attachments/assets/a83ef06f-8f77-4746-921f-0aee567b3e9d




