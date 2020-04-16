#!/usr/bin/env python

import rospy
import numpy as np
from robot_simulation.msg import EndCondition
# input target x, y, z, theta_t
L1 = 0.5
L2 = 0.5
q1 = np.deg2rad(10) 
q2 = np.deg2rad(20)
q3 = np.deg2rad(0)

# output: q1, q2, q3, v
def state_solver():
  pass


def Jacobian(L1, L2, q1, q2, q3):
  # Jacobian of manipulators
  J = np.array(([-(L1*np.cos(q2) + L2*np.cos(q2+q3))*np.sin(q1), -(L1*np.sin(q2) + L2*np.sin(q2+q3))*np.cos(q1), -L2*np.sin(q2+q3)*np.cos(q1)],
                [(L1*np.cos(q2) + L2*np.cos(q2+q3))*np.cos(q1), -(L1*np.sin(q2) + L2*np.sin(q2+q3))*np.sin(q1), -L2*np.sin(q2+q3)*np.sin(q1)],
                [0, L1*np.cos(q2)+L2*np.cos(q2+q3), L2*np.cos(q2+q3)]))
  return J

# output: q1_dot, q2_dot, q3_dot
# input: q1, q2, q3, vx, vy, vz, J
def q_dot(v, J):
  
  q_dot = np.linalg.solve(J, v)
  
  return q_dot

def publisher():
  pub = rospy.publisher('/end_conditions', EndConditions, queue_sldkfja=10)
  

