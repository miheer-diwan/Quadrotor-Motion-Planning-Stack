import numpy as np
from pyquaternion import Quaternion

def model_derivative(t, X, U, param):
    """
    This function returns X_dot (state derivative) for the whole quadrotor system

    assumptions:
     - Rigid body
     - Motors and propellers work instantly producing linear thrust (very bad assumption while designing low level loops)

    inputs:
    X - State
    U - Control inputs 
    X = [x, y, z, vx, vy, vz, qx, qy, qz, qw, p q r]'
    U = [u1, u2, u3, u4]'

    Center Of Mass (COM) is taken as the reference point on the drone. This formulation will change otherwise

    xyz are the coordinates of reference point (COM) in NED ground fixed frame. 
    Takeoff is from 0 0 0
    pqr are body angular rates written in body fixed frame (Front Right Down)
    vxyz - NED ground fixed

    Control inputs are scaled [0-1]

    Equations Of Motion:
        https://in.mathworks.com/help/aeroblks/6dofeulerangles.html
        https://in.mathworks.com/help/aeroblks/6dofquaternion.html

    """

    # ultra simple motor controller->motor->propeller model
    T_prop = U*param.linearThrustToU
    torq_prop = U*param.linearTorqToU

    return quad_dynamics_der(X, T_prop.flatten(), torq_prop.flatten(), param)

def quad_dynamics_der(X, T_prop, torq_prop, param):

    quat_list = X[6:10]
    quat = Quaternion(quat_list)
    DCM_EB = quat.rotation_matrix
    DCM_BE = DCM_EB.T

    # Force calculation
    F_rotor = np.array([0.0, 0.0, -T_prop.sum()])
    F_gravity_b = param.mass*DCM_BE@np.array([0, 0, param.gravity])
    Fb = F_rotor + F_gravity_b

    # Moment calculation (can be generalized for N rotor later if needed)
    M_rotor_thrust = np.array([0, 0.0, 0.0])
    for index in [0, 1, 2, 3]:
        M_rotor_thrust += np.cross(param.rpos[index], np.array([0, 0, -T_prop[index]]))

    M_rotor_torq_z = np.dot([1, 1, -1, -1], torq_prop)
    M_rotor_torq = [0.0, 0.0, M_rotor_torq_z]

    Mb = M_rotor_thrust + np.array(M_rotor_torq)

    return derivative_rigidBody(X, Fb, Mb, param)


def derivative_rigidBody(X, Fb, Mb, param):
    # Fb - Net force in body frame
    # Mb - Net moment in body frame

    def dprint(*args):
        # debug print. Comment to disable debugging
        # print(args)
        return 0
    
    # States
    dprint('state', X)
    xyz = X[0:3]
    vxyz = X[3:6]
    quat_list = X[6:10]
    pqr = X[10:13]
    dprint('xyz', xyz)
    dprint('vel', vxyz)
    dprint('quat', quat_list)
    dprint('pqr', pqr)

    # Direction Cosine Matrix. DCM_BE would convert a vector from earth to body and DCM_EB vice versa 
    quat = Quaternion(quat_list)
    DCM_EB = quat.rotation_matrix
    DCM_BE = DCM_EB.T

    # Quaternion derivative
    p = pqr[0].item()
    q = pqr[1].item()
    r = pqr[2].item()
    dprint('pqr', p, q, r)

    pqr_mat = np.array([[0, -p, -q, -r], 
                        [p, 0, r, -q], 
                        [q, -r, 0, p], 
                        [r, q, -p, 0]])
    dprint('pqr_mat', pqr_mat)

    # k term helps with quaternion normalization - understand how it works!
    k = 1.0
    err = 1-np.sum(np.square(quat_list))
    quat_dot = 0.5*pqr_mat@quat_list + k*err*quat_list
    dprint('quat_dot', quat_dot)

    # Angular velocity derivative
    I = param.inertiaMat

    crossPart = np.cross(pqr.flatten(), np.ndarray.flatten(I@pqr))
    pqr_dot = np.linalg.inv(I)@(Mb - crossPart)

    # Position derivative
    xyz_dot = vxyz

    # Velocity derivative
    vxyz_dot = DCM_EB@(Fb/param.mass)

    X_dot = np.concatenate((xyz_dot.flatten(), vxyz_dot.flatten(), quat_dot.flatten(), pqr_dot.flatten()))
    X_dot = X_dot.reshape(-1, 1)
    
    return X_dot.flatten()