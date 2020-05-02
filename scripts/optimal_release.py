#!/usr/bin/env python

import rospy
import sys
from scipy.optimize import minimize
import numpy as np
from robot_simulation.msg import ReleaseStates

L1 = 0.50
L2 = 0.50
height_to_base_joint = 1.250
g = 9.81
msg = ReleaseStates()

def find_optimal_release_point(d, h_f):
    rospy.init_node('release_point_solver')
    pub = rospy.Publisher('/release_states', ReleaseStates, queue_size=1)

    # Value bounds for each variable
    bnds = [[-np.pi/2, np.pi/2], [0, np.pi], [0, None]]

    sol = minimize(cost_function, [0, .1, 2], args=(d, h_f, L1, L2), bounds=bnds)
    velocity = (d - (L1*np.sin(sol.x[0]) + L2*np.sin(sol.x[0]+sol.x[1]))) / (sol.x[2]*np.cos(sol.x[0]+sol.x[1]))
    joint_angles = np.array([sol.x[0], sol.x[1]])

    print('theta_1 = ' + str(sol.x[0]))
    print('theta_2 = ' + str(sol.x[1]))
    print('velocity = ' + str(sol.x[2]))
    
    joint_velocities = find_joint_velocity(joint_angles[0], joint_angles[1], velocity)
    coeff = find_trajectory_plan(joint_velocities, joint_angles)

    print(coeff)

###############################################################################
# Finds joint velocities given projectile velocity 
###############################################################################
def find_joint_velocity(th1, th2, v):
    J = np.array(([L1*np.cos(th1) + L2*np.cos(th1 + th2), L2*np.cos(th1 + th2)],
                   [L1*np.sin(th1) + L2*np.sin(th1 + th2), L2*np.sin(th1 + th2)]))
    vx = v*np.sin(th1 + th2 + np.pi/2)
    vy = v*np.cos(th1 + th2 + np.pi/2)
    B = np.array([vx, vy])

    retVal = (np.linalg.pinv(J).dot(B))
    return retVal

def find_trajectory_plan(q, th):
    # Use jacobian to find joint velocities
    B = np.array([0, 0, th[0], q[0]])
    A = cubic(0, 2)

    retVal = np.linalg.pinv(A).dot(B)
    
    return retVal

def cubic(t0, tf):
    retVal = np.array([[1, t0, t0**2, t0**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*tf, 3*tf**2]])
    return retVal

#########################################
# Cost function which minimizes velocity
#########################################
def cost_function(x, d, h_f, L1, L2):
    th1 = x[0]
    th2 = x[1]
    t = x[2]

    th_t = th1 + th2 + np.pi/2

    h_i = height_to_base_joint - (L1*np.cos(th1) + L2*np.cos(th_t))
    dt = d - (L1*np.sin(th1) + L2*np.sin(th_t))

    vx = dt/(t*np.cos(th_t))
    vy = (h_f - h_i + g/2*t**2)/(t*np.sin(th_t)) 

    return vx**2 + vy**2

if __name__=="__main__":
    if (sys.argv < 2):
        print('usage: my_node distance h_final')
    else:
        find_optimal_release_point(float(sys.argv[1]), float(sys.argv[2]))