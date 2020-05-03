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
    self.q1 = 0
    self.q2 = 0
    self.J = np.array(0)
    self.msg = ReleaseStates()
    self.v = np.array([5, 5])
    self.pub = rospy.Publisher("/end_states", ReleaseStates, queue_size=10)

  # output: q1, q2, q3, v
  def state_solver(self, x, y, z):
    
    x = float(x)
    y = float(y)
    z = float(z)

    self.msg.angle = np.arctan( y / x )

    #Parameters
    l1 = 0.3 #Link1 Length
    l2 = 0.3255 #Link2 Length
    g = 9.81 #Acceleration due to gravity
    h1 = 1.00 #Height of Joint1 starting from ground(y=0)
    def release_point(D,h2,alpha):
        #D is the distance along the ground between target and Joint1
        #h2 is the height of the target from ground (y=0)
        #alpha is the slope of the target plane

        q1dbag = [] #q1dot list
        q2dbag = [] #q2dot list
        vxbag = [] #x velocity list
        vybag = [] #y velocity list
        q1list = [] #q1 list
        q2list = [] #q2 q2list
        q1bound = 46 #Upperbound of q1 iteration in degrees
        q2bound = 91 #Upperbound of q2 iteration in degrees
        for i in range(1,q1bound): #Iterating q1 from 1 to 45 degrees (Excluded 0 to avoid singularity of matrix)
          for j in range(1,q2bound): #Iterating q2 from 1 to 90 degrees (Excluded 0 to avoid singularity of matrix)
            q1 = np.pi*i/180 #degrees to radians
            q2 = np.pi*j/180 #degrees to radians
            xin = l1*np.sin(q1) + l2*np.sin(q1+q2) #Release point x-coordinate
            yin = h1 - (l1*np.cos(q1)+l2*np.cos(q1+q2)) #Release point y-coordinate
            vxin = ((0.5*g*(D-xin)**2)/((h2-yin)+(D-xin)*(1/ np.tan(np.deg2rad(alpha)))))**0.5 #x velocity calculation
            vyin = g*(D-xin)/vxin - vxin/np.tan(np.deg2rad(alpha)) #y velocity calculation
            vxbag.append(vxin)
            vybag.append(vyin)

            J = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2), l2*np.cos(q1+q2)],[l1*np.sin(q1)+l2*np.sin(q1+q2), l2*np.sin(q1+q2)]]) #Jacobian
            Jinv = np.linalg.inv(J) #Inverse of Jacobian
            qdots = np.dot(Jinv, np.array([[vxin],[vyin]])) #Joint angular velocities q1dot q2dot
            q1dbag.append(qdots[0,0]) #Collecting q1dots
            q2dbag.append(qdots[1,0]) #collecting q2dots
            q1list.append(i) #collecting q1's for convenience
            q2list.append(j) #collecting q2's for convenience

        #Finding the index where maximum of q1dot and q2dot is minimum
        maxdot = [max(abs(q1dbag[i]),abs(q2dbag[i])) for i in range((q1bound-1)*(q2bound-1))] #Finding maximum magnitude among q1dot and q2dot
        minmaxpos = maxdot.index(min(maxdot)) #Finding index where the maximum is minimum
        return np.array([q1list[minmaxpos],q2list[minmaxpos],q1dbag[minmaxpos],q2dbag[minmaxpos]]) #Returning q1,q2,q1dot,q2dot as an array

    states = release_point(np.sqrt(x**2 + y**2), 0.3, 30)

    self.msg.q1 = np.deg2rad(states[0])*1.05
    self.msg.q2 = np.deg2rad(states[1])*1.05
    self.msg.q1_dot = states[2]
    self.msg.q2_dot = states[3]

    self.pub.publish(self.msg)
    rospy.loginfo(self.msg)
    # self.Jacobian()

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

        
