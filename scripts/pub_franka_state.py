#! /usr/bin/env python3
'''
publish franka state using ros built in message types
for MATLAB
'''
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray


class pub_franka_state():
    T_O_ee = None
    franka_state_msg = Float64MultiArray()

    def __init__(self):
        # initialize node
        rospy.init_node('publish_franka_state', anonymous=True)
        # subscriber
        rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.ee_callback)
        # publisher
        self.franka_state_pub = rospy.Publisher('franka_state_custom', Float64MultiArray, queue_size=5)

        print("publishing franka state ...")
        while not rospy.is_shutdown():
            if self.T_O_ee is not None:
                self.franka_state_msg.data = self.T_O_ee.flatten()
                self.franka_state_pub.publish(self.franka_state_msg)
                # print(T_O_ee)
            else:
                pass

    def ee_callback(self, msg):
        EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
        self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
    pub_franka_state()
