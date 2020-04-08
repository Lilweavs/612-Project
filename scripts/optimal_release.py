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

    bnds = [[-45, 45], [0, 90], [0, None]] #Bounds for each variable
    sol = minimize(cost_function, [10, 10, 2], args=(d, h_f, L1, L2), bounds=bnds)

    msg.velocity = (d - (L1*np.sin(sol.x[0]) + L2*np.sin(sol.x[0]+sol.x[1]))) / (sol.x[2]*np.cos(sol.x[0]+sol.x[1]))
    msg.theta1 = sol.x[0]
    msg.theta2 = sol.x[1]

    ## Outputs joint positions, and velocity

    print(sol.x)

def cost_function(x, d, h_f, L1, L2):
    th1 = np.deg2rad(x[0])
    th2 = np.deg2rad(x[1])
    t = x[2]

    th_t = th1 + th2

    h_i = height_to_base_joint - (L1*np.cos(th1) + L2*np.cos(th_t))
    dt = d - (L1*np.sin(th1) + L2*np.sin(th_t))

    vx = dt/(t*np.cos(th_t))
    vy = (h_f - h_i + g/2*t**2)/(t*np.sin(th_t)) 

    return vx**2 + vy**2


if __name__=="__main__":
    if (sys.argv < 2):
        print('usage: my_node distance h_final')
    else:
        find_optimal_release_points(float(sys.argv[1]), float(sys.argv[2]))