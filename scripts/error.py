#!/usr/bin/env python
import rospy  
import numpy as np  
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from robot_simulation.msg import ReleaseStates


class Error:

    def __init__(self):
        self.sub = rospy.Subscriber('/simple_model/joint_states', JointState, self.joint_error)
        self.q1_max = 0
        self.q2_max = 0
        self.e1_max = 0
        self.e2_max = 0
        # self.sub = rospy.Subscriber('/simple_model/joint_states', JointState, self.latch_release )
        # self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_path)
        #self.sub = rospy.Subscriber('/clock', Clock, self.get_time_callback)
        #rospy.Timer(rospy.Duration(0.1), publish_path)

    def joint_error(self, msg):
        if (msg.velocity[1] > self.q1_max):
            self.q1_max = msg.velocity[1]
            self.e1_max = msg.effort[1]
            rospy.loginfo('q1_max = ' + str(self.q1_max))
            rospy.loginfo('e1_max = ' + str(self.e1_max))
        if (msg.velocity[3] > self.q2_max):
            self.q2_max = msg.velocity[3]
            self.e2_max = msg.effort[3]
            rospy.loginfo('q2_max = ' + str(self.q2_max))
            rospy.loginfo('e2_max = ' + str(self.e2_max))

if __name__=="__main__":
    rospy.init_node('Error')
    n = Error()
    rospy.spin()