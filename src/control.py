import numpy as np
import math
from pyquaternion import Quaternion
from numpy.linalg import norm
import scipy

class quad_control:
    def __init__(self):

        # CONTROLLER PROPERTIES AND GAINS
        dt = 0.010
        filter_tau = 0.04
        self.dt = dt

        # tello params
        self.param_mass = 0.08
        self.linearThrustToU = self.param_mass*9.81*2/4

        maxAcc = 5.
        maxVel = 10.
        # maxAng = 30.*3.14159/180.
        self.maxRate = 1.5
        maxAct = 0.3

        minAcc = -maxAcc
        minVel = -maxVel
        # minAng = -maxAng
        self.minRate = -self.maxRate
        minAct = -maxAct
        
        # EDIT PID GAINS HERE! (kp, ki, kd, filter_tau, dt, dim = 1, minVal = -1, maxVal = 1)
        # NED position controller. EDIT GAINS HERE
        self.x_pid = pid(1, 0, 0, filter_tau, dt, minVal = minVel, maxVal=maxVel)
        self.y_pid = pid(1, 0, 0, filter_tau, dt, minVal = minVel, maxVal=maxVel)
        self.z_pid = pid(0.1, 0.01, 0.1, filter_tau, dt, minVal = minVel, maxVal=maxVel)

        # NED velocity controller. EDIT GAINS HERE
        self.vx_pid = pid(2.0, 0.5, 0.5, filter_tau, dt, minVal = minAcc, maxVal=maxAcc)
        self.vy_pid = pid(2.0, 0.5, 0.5, filter_tau, dt, minVal = minAcc, maxVal=maxAcc)
        self.vz_pid = pid(20.0, 0.1, 0.1, filter_tau, dt, minVal = minAcc, maxVal = maxAcc)
        
        # Quaternion based P Controller. Output is desired angular rate. tau is time constant of closed loop
        self.tau_angle = 0.3
        self.angle_sf = np.array((1, 1, 0.4)) # deprioritize yaw control using this scale factor

        # Angular velocity controller
        kp_angvel = 6.
        self.p_pid = pid(kp_angvel, 0, kp_angvel/15., filter_tau, dt, minVal = minAct, maxVal = maxAct)
        self.q_pid = pid(kp_angvel, 0, kp_angvel/15., filter_tau, dt, minVal = minAct, maxVal = maxAct)
        self.r_pid = pid(kp_angvel, 0, kp_angvel/15, filter_tau, dt, minVal = minAct, maxVal = maxAct)

        # For logging
        self.current_time = 0.
        self.timeArray = 0
        self.controlArray = np.array([0., 0, 0, 0])

    def step(self, X, WP, VEL_SP, ACC_SP):
        """
        Quadrotor position controller
        Assumptions:
            - Everything is linear
            - No disturbance
            - When everythng is linearized at hover, quad dynamics become neatly decoupled as well
            - I am going to run all the control loops at the same rate. But gains will ensure that the outer loop has lower bandwidth than the inner loop
            - Good for low speed scenarios. Probably bad for high speed scenarios
            - Full state feedback is available

        Limitations:
            - Integrator reset not handled
            - Integrator anti-windup not handled

        X - follows the same conventions as used in dynamics simulation
        WP - np.array([X, Y, Z, Yaw]) NED position and yaw in radians
        VEL_SP - np.array(vx, vy, vz) NED velocity setpoint from motion profile
        ACC_SP - np.array(acc_x, acc_y, acc_z) NED acceleration setpoint from motion profile

        U - Returns numpy 4x1 array (4 motor normalized control input)

        position->velocity->acceleration(euler angles)->angular rate->mixer->motors

        Architecture is similar to px4 https://docs.px4.io/main/en/flight_stack/controller_diagrams.html

        https://www.motioncontroltips.com/faq-what-is-piv-servo-control/

        """
        # EXTRACT STATES
        xyz = X[0:3]
        vxyz = X[3:6]
        quat_list = X[6:10]
        pqr = X[10:13]

        quat = Quaternion(quat_list)
        ypr = quat.yaw_pitch_roll
        yaw = ypr[0]
        pitch = ypr[1]
        roll = ypr[2]

        DCM_EB = quat.rotation_matrix
        DCM_BE = DCM_EB.T

        # NED POSITION CONTROLLER
        vx_ned_sp = VEL_SP[0] + self.x_pid.step(WP[0], xyz[0])
        vy_ned_sp = VEL_SP[1] + self.y_pid.step(WP[1], xyz[1])
        vz_ned_sp = VEL_SP[2] + self.z_pid.step(WP[2], xyz[2])

        vxyz_sp = np.array([vx_ned_sp, vy_ned_sp, vz_ned_sp])

        # # NED VELOCITY CONTROLLER x, y, z
        acc_x_sp = ACC_SP[0] + self.vx_pid.step(vxyz_sp[0], vxyz[0])
        acc_y_sp = ACC_SP[1] + self.vy_pid.step(vxyz_sp[1], vxyz[1])
        acc_z_sp = ACC_SP[2] + self.vz_pid.step(vxyz_sp[2], vxyz[2]) 

        # ACCELERATION SETPOINT TO QUATERNION SETPOINT

        # mass specific force to be applied by the actuation system
        f_inertial = np.array((acc_x_sp, acc_y_sp, acc_z_sp)) - np.array((0., 0., 9.81))

        rotationAxis = np.cross(np.array((0., 0., -1.)), f_inertial/norm(f_inertial))
        rotationAxis += np.array((1e-3, 1e-3, 1e-3)) # Avoid numerical issue

        sinAngle = norm(rotationAxis)
        rotationAxis = rotationAxis / norm(rotationAxis)

        cosAngle = np.dot( np.array((0., 0., -1.)), f_inertial/norm(f_inertial) )

        angle = math.atan2(sinAngle, cosAngle)

        quat_wo_yaw = Quaternion(axis=rotationAxis, radians=angle)

        quat_yaw = Quaternion(axis=np.array((0., 0., 1)), radians=WP[3])

        # I dont think the order of multiplication matters in this special case. check later TODO
        # Also multiplication can even be done in the first place as they share the same NED basis
        quat_sp = quat_wo_yaw * quat_yaw

        # QUATERNION P CONTROLLER (simplified version of px4 implementation) 8)
        # https://github.com/PX4/PX4-Autopilot/blob/1c1f8da7d9cc416aaa53d76254fe08c2e9fa65e6/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp#L91

        # quat_sp = Quaternion(-0.183, 0.113, 0.108, 0.971)
        err_quat = quat.inverse*quat_sp
        
        pqr_sp = 2./self.tau_angle*np.sign(err_quat.w)*np.array((err_quat.x, err_quat.y, err_quat.z))
        # pqr_sp = np.array((0.1, 0.2, -0.2))

        pqr_sp = np.multiply(pqr_sp, self.angle_sf)
        pqr_sp = pqr_sp.clip(self.minRate, self.maxRate)

        # ANGULAR VELOCITY
        tau_x = self.p_pid.step(pqr_sp[0], pqr[0])
        tau_y = self.q_pid.step(pqr_sp[1], pqr[1])
        tau_z = self.r_pid.step(pqr_sp[2], pqr[2])

        # ROTOR THRUST. Cheating a bit here making use of tello parameters
        netSpecificThrustFromRotors = norm(f_inertial) # N/kg
        netThrust = netSpecificThrustFromRotors * self.param_mass

        thrustPerRotor = netThrust/4.
        throttle = thrustPerRotor/self.linearThrustToU

        # MIXER
        u1 = throttle - tau_x + tau_y + tau_z
        u2 = throttle + tau_x - tau_y + tau_z
        u3 = throttle + tau_x + tau_y - tau_z
        u4 = throttle - tau_x - tau_y - tau_z

        U = np.array([u1, u2, u3, u4])
        U = U.clip(0.0, 1.0)

        # Logger
        self.controlArray = np.vstack((self.controlArray, np.array((throttle, tau_x, tau_y, tau_z))))
        self.timeArray = np.append(self.timeArray, self.current_time)
        self.current_time+=self.dt

        loggedDict = {'control_time': self.timeArray,
                  'control_premix': self.controlArray}
        
        scipy.io.savemat('./log/control.mat', loggedDict)


        return U

class pid:
    def __init__(self, kp, ki, kd, filter_tau, dt, dim = 1, minVal = -1, maxVal = 1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.minVal = minVal
        self.maxVal = maxVal
        self.filter_tau = filter_tau
        self.dt = dt

        self.minVal = minVal
        self.maxVal = maxVal

        if dim == 1:
            self.prev_filter_val = 0.0
            self.prev_err = 0.0
            self.prev_integral = 0.0
        else:
            self.prev_err = np.zeros(dim, dtype="double")
            self.prev_filter_val = np.zeros(dim, dtype="double")
            self.prev_integral = np.zeros(dim, dtype="double")
    
    def step(self, dsOrErr, current_state = None):

        # Error
        if current_state is None:
            err = dsOrErr
        else:
            desired_state = dsOrErr
            err = desired_state - current_state

        # Error Derivative and filtering
        err_der = (err-self.prev_err)/self.dt

        # Forward euler discretization of first order LP filter
        alpha = self.dt/self.filter_tau
        err_der_filtered = err_der*alpha + self.prev_filter_val*(1-alpha)

        # Integral
        err_integral = err*self.dt + self.prev_integral

        # Raw Output
        out = self.kp*err + self.kd*err_der_filtered + self.ki*err_integral

        # NaN check
        if math.isnan(out):
            print('err', err)
            print(err_integral)
            print(err_der)
            print('Make sure waypoints are not nan. If you still get this error, contact your TA.')
            if current_state is None:
                print('Error is directly provided to the PID')
            else:
                print('desired - ', desired_state)
                print('current - ', current_state)
            raise Exception('PID blew up :( out is nan')

        # Update the internal states
        self.prev_err = err
        self.prev_filter_val = err_der_filtered
        self.prev_integral = err_integral

        # Integral anti-windup. Clamp values
        self.prev_integral = np.clip(self.prev_integral, self.minVal, self.maxVal)

        # Clip the final output
        out = np.clip(out, self.minVal, self.maxVal)

        # Inf check
        if math.isinf(out):
            raise Exception('PID output is inf')
        
        return out
