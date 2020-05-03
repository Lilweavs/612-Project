#!/usr/bin/env python
import rospy  
import numpy as np  
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def callback(msg):
    # rospy.loginfo(msg.position[1] + msg.position[3])
    if (msg.position[1] +  msg.position[3]) > np.pi/64:
        release_latch(np.pi/2)
    else:
        release_latch(-0.05)

def release_latch(val):
    pub = rospy.Publisher('/simple_model/end_effector_to_latch_position_controller/command', Float64, queue_size=10)
    pub.publish(val)

if __name__ == "__main__":
    rospy.init_node("Latch_Release")
    sub = rospy.Subscriber('/simple_model/joint_states', JointState, callback)
    rospy.spin()

