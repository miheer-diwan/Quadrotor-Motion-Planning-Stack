# DJI Tello properties

# SI units unless specified otherwise

# Benotsmane, R.; Vásárhelyi, J. Towards Optimization of Energy Consumption of Tello Quad-Rotor with Mpc Model Implementation. Energies 2022, 15, 9207. https://doi.org/10.3390/en15239207 

import numpy as np
mass = 0.08
Ixx = 0.0097
Iyy = 0.0097
Izz = 0.017
# inertia matrix as given in https://in.mathworks.com/help/aeroblks/6dofeulerangles.html
inertiaMat = np.diag([Ixx, Iyy, Izz])

rotorDragCoeff = 0.08
rotorLiftCoeff = 1
halfDiag = 0.06

gravity = 9.81

# Rotor position
rpos = np.array([
    [1, 1, 0],
    [-1, -1, 0],
    [1, -1, 0],
    [-1, 1, 0]
])*halfDiag/np.sqrt(2.0)

# linear control input to thrust mapping
#  This is totally guess work for now
#  Assuming, the drone runs at half throttle during hover 
linearThrustToU = mass*gravity*2/4
linearTorqToU = linearThrustToU/rotorLiftCoeff*rotorDragCoeff