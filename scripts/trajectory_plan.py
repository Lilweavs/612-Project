#!/usr/bin/env python
import rospy  
import numpy as np  
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from robot_simulation.msg import ReleaseStates
t_total = 2


class Trajectory_Planner:

    def __init__(self):
        self.a0 = 0
        self.a1 = 0
        self.a2 = 1.398
        self.a3 = -0.58528
        self.t0 = rospy.Time.to_sec(rospy.get_rostime())
        self.t1 = rospy.get_rostime()
        self.pub_joint1 = rospy.Publisher('/simple_model/base_to_first_joint_position_controller/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/simple_model/first_to_second_joint_position_controller/command', Float64, queue_size=10)
        self.pub_latch = rospy.Publisher('/simple_model/end_effector_to_latch_position_controller/command', Float64, queue_size=10)
        self.sub = rospy.Subscriber('/simple_model/joint_states', JointState, self.latch_release )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_path)
        #self.sub = rospy.Subscriber('/clock', Clock, self.get_time_callback)
        #rospy.Timer(rospy.Duration(0.1), publish_path)

    def latch_release(self, msg):
        rospy.loginfo(msg.position[0])

    def get_inital_time(self):
        self.t0 = rospy.get_rostime()

    def wait(self):
        pass
        #print('Waiting for command')
        #rospy.wait_for_message('/run', Float64)
        #timer = rospy.Timer(rospy.Duration(0.1), self.publish_path)
        #self.t0 = rospy.Time.to_sec(rospy.get_rostime())
        #def publish_path(msg):
        #    print(msg)


    def get_time_callback(self, msg):
        pass
        #rospy.loginfo(msg.nsec)
        #rospy.loginfo(self.t0)


    def publish_path(self, msg):

        elapsed_time = rospy.Time.to_sec(msg.current_real) - self.t0
        val = self.a0 + self.a1*elapsed_time + self.a2*elapsed_time**2 + self.a3*elapsed_time**3
        print('t = '+str(elapsed_time)+' q = '+str(val))
        self.pub.publish(val)
        if elapsed_time > 2:
            print('shutting down')
            self.timer.shutdown()


def talker():
    pub = rospy.Publisher('/simple_model/base_to_firs_link_joint_state_publisher/command', Float64) 


if __name__=="__main__":
    rospy.init_node('trajectory_planner')
    rospy.sleep(1)
    n = Trajectory_Planner()
    n.wait()
    rospy.spin()