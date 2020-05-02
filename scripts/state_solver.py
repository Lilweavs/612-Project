#!/usr/bin/env python

import rospy
import numpy as np
import sys
from robot_simulation.msg import ReleaseStates
# input target x, y, z, theta_t

class StateSolver():

  def __init__(self):
    rospy.init_node("State_Solver")
    self.L1 = 0.3
    self.L2 = 0.3
    self.q1 = np.pi/8
    self.q2 = np.pi/8
    self.J = np.array(0)
    self.msg = ReleaseStates()
    self.v = np.array([2, 2])
    self.pub = rospy.Publisher("/end_states", ReleaseStates, queue_size=10)

  # output: q1, q2, q3, v
  def state_solver(self, x, y, z):
    
    x = float(x)
    y = float(y)
    z = float(z)

    self.msg.angle = np.arctan( y / x )

    self.msg.q1 = self.q1
    self.msg.q2 = self.q2

    self.Jacobian()

  # Jacobian of manipulators
  def Jacobian(self):
    
    L1 = self.L1
    L2 = self.L2
    q1 = self.msg.q1
    q2 = self.msg.q2

    # self.J = np.array(([-(L1*np.cos(q1) + L2*np.cos(q1+q2)), -(L1*np.sin(q1) + L2*np.sin(q1+q2)), -L2*np.sin(q1+q2)],
    #               [(L1*np.cos(q1) + L2*np.cos(q1+q2)), -(L1*np.sin(q1) + L2*np.sin(q1+q2)), -L2*np.sin(q1+q2)],
    #               [0, L1*np.cos(q1) + L2*np.cos(q1+q2), L2*np.cos(q1+q2)]))

    self.J = np.array([[L1*np.cos(q1) + L2*np.cos(q1 + q2), L2*np.cos(q1 + q2)],
                       [L1*np.sin(q1) + L2*np.sin(q1 + q2), L2*np.sin(q1 + q2)]])

    self.q_dot()

  # output: q1_dot, q2_dot, q3_dot
  # input: q1, q2, q3, vx, vy, vz, J
  def q_dot(self):
  
    q_dot = np.linalg.solve(self.J, self.v)

    self.msg.q1_dot = q_dot[0]
    self.msg.q2_dot = q_dot[1]
    # self.msg.q3_dot = q_dot[2]

    self.pub.publish(self.msg)
    rospy.loginfo(self.msg)
  
if __name__=="__main__":
    if (sys.argv < 5):  
        print('usage: my_node x y z')
    else:
      n = StateSolver()
      rate = rospy.Rate(5)
      while not rospy.is_shutdown():
        connections = n.pub.get_num_connections()
        rospy.loginfo('Connections: %d', connections)
        if connections > 0:
          n.state_solver(sys.argv[1], sys.argv[2], sys.argv[3])
          rospy.loginfo('Published')
          break
        rate.sleep()

        
