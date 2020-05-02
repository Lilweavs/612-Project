#!/usr/bin/env python

import rospy
import numpy as np
import sys
from robot_simulation.msg import ReleaseStates
# input target x, y, z, theta_t

class StateSolver():

  def __init__(self):
    self.L1 = 0.5
    self.L2 = 0.5
    self.q1 = 0
    self.q2 = np.pi/8
    self.q3 = np.pi/8
    self.J = np.array(0)
    self.msg = ReleaseStates()
    self.v = np.array([1, 0, -1])
    self.pub = rospy.Publisher('/end_states', ReleaseStates, queue_size=1)

  # output: q1, q2, q3, v
  def state_solver(self, x, y, z, angle):
    '''
    self.msg.q1 = 0
    self.msg.q2 = 0
    self.msg.q3 = 0
    '''
    self.Jacobian()

  # Jacobian of manipulators
  def Jacobian(self):
    
    L1 = self.L1
    L2 = self.L2
    q1 = self.q1
    q2 = self.q2
    q3 = self.q3

    self.J = np.array(([-(L1*np.cos(q2) + L2*np.cos(q2+q3))*np.sin(q1), -(L1*np.sin(q2) + L2*np.sin(q2+q3))*np.cos(q1), -L2*np.sin(q2+q3)*np.cos(q1)],
                  [(L1*np.cos(q2) + L2*np.cos(q2+q3))*np.cos(q1), -(L1*np.sin(q2) + L2*np.sin(q2+q3))*np.sin(q1), -L2*np.sin(q2+q3)*np.sin(q1)],
                  [0, L1*np.cos(q2) + L2*np.cos(q2+q3), L2*np.cos(q2+q3)]))
    self.q_dot()

  # output: q1_dot, q2_dot, q3_dot
  # input: q1, q2, q3, vx, vy, vz, J
  def q_dot(self):
  
    q_dot = np.linalg.solve(self.J, self.v)/(2*np.pi)*60

    self.msg.q1_dot = q_dot[0]
    self.msg.q2_dot = q_dot[1]
    self.msg.q3_dot = q_dot[2]

    rospy.loginfo(self.msg)
    self.pub.publish(self.msg)
  
if __name__=="__main__":
    if (sys.argv < 5):  
        print('usage: my_node x y z theta_t')
    else:
      rospy.init_node("Blain_Likes_Feet")
      n = StateSolver()
      n.state_solver(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
        
