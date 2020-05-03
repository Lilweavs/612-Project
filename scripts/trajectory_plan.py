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
        self.tf = 1
        self.tim_res = 0.002
        self.t0 = rospy.Time.to_sec(rospy.get_rostime())
        self.states = ReleaseStates()
        self.coeff = []
        self.timer = None
        self.pub_joint1 = rospy.Publisher('/simple_model/base_to_first_joint_position_controller/command', Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/simple_model/first_to_second_joint_position_controller/command', Float64, queue_size=10)
        self.pub_latch = rospy.Publisher('/simple_model/end_effector_to_latch_position_controller/command', Float64, queue_size=10)
        self.pub_base = rospy.Publisher('/simple_model/base_joint_position_controller/command', Float64, queue_size=10)

    def latch_release(self, msg):
        rospy.loginfo(msg.position[3])
        if (msg.position[1]) > np.pi/32:
            self.pub_latch.publish(np.pi/2)

    def cubic(self, t0, tf):
        retVal = np.array([[1, t0, t0**2, t0**3],
                            [0, 1, 2*t0, 3*t0**2],
                            [1, tf, tf**2, tf**3],
                            [0, 1, 2*tf, 3*tf**2]])
        return retVal

    def find_trajectory_plan(self):
        # Use jacobian to find joint velocities
        B1 = np.array([0, 0, self.states.q1, self.states.q1_dot])
        B2 = np.array([0, 0, self.states.q2, self.states.q2_dot])
        B3 = np.array([0, 0, -self.states.angle, 0])
        A = self.cubic(0, self.tf)
        A1 = self.cubic(0, 2)

        retVal = [np.linalg.pinv(A).dot(B1), np.linalg.pinv(A).dot(B2), np.linalg.pinv(A1).dot(B3)]
    
        return retVal

    def wait(self):
        print('Waiting for command...')
        self.states = rospy.wait_for_message('/end_states', ReleaseStates)
        print('End States Received!')
        rospy.loginfo(self.states)
        self.coeff = self.find_trajectory_plan()
        print('Yeet!!!')

        self.t0 = rospy.Time.to_sec(rospy.get_rostime())
        self.timer = rospy.Timer(rospy.Duration(self.tim_res), self.publish_angle)
        # self.timer = rospy.Timer(rospy.Duration(self.tim_res), self.publish_path)
        #self.t0 = rospy.Time.to_sec(rospy.get_rostime())
        #def publish_path(msg):
        #    print(msg)

    def publish_angle(self, msg):

        t = rospy.Time.to_sec(msg.current_real) - self.t0
        val = self.coeff[2][0] + self.coeff[2][1]*t + self.coeff[2][2]*t**2 + self.coeff[2][3]*t**3
        self.pub_base.publish(val)
        if t >= (2):
            print('Trajectory Complete')
            self.timer.shutdown()
            self.t0 = rospy.Time.to_sec(rospy.get_rostime())
            self.timer = rospy.Timer(rospy.Duration(self.tim_res), self.publish_path)


    def publish_path(self, msg):

        t = rospy.Time.to_sec(msg.current_real) - self.t0
        q1d = self.coeff[0][0] + self.coeff[0][1]*t + self.coeff[0][2]*t**2 + self.coeff[0][3]*t**3
        q2d = self.coeff[1][0] + self.coeff[1][1]*t + self.coeff[1][2]*t**2 + self.coeff[1][3]*t**3
        print('t = ' + str(t) + ' q1d = ' + str(q1d) + ', q2d = ' + str(q2d))
        self.pub_joint1.publish(q1d)
        self.pub_joint2.publish(q2d)
        if t >= (self.tf):
            print('Trajectory Complete')
            self.timer.shutdown()
        
    def cleanup(self):
        print('Reseting Robot Arm')
        self.pub_joint1.publish(0)
        self.pub_joint2.publish(0)
        self.pub_latch.publish(0)
        self.pub_base.publish(0)


if __name__=="__main__":
    rospy.init_node('trajectory_planner')
    rospy.sleep(1)
    n = Trajectory_Planner()
    rospy.on_shutdown(n.cleanup)
    n.wait()
    rospy.spin()