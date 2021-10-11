#! /usr/bin/env python3
'''
record robot data
'''
import csv
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import Bool


def ee_callback(msg):
    EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
    global T_O_ee
    T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


def OCT_clk_ctrl_callback(msg):
    global OCT_clk_ctrl
    OCT_clk_ctrl = msg.data


def force_callback(msg):
    global Fz
    Fz = msg.wrench.force.z


rospy.Subscriber('franka_state_controller/franka_states', FrankaState, ee_callback)
rospy.Subscriber('/franka_state_controller/F_ext', WrenchStamped, force_callback)
rospy.Subscriber('OCT_clk_ctrl', Int16, OCT_clk_ctrl_callback)

T_O_ee = None
isContact = 0
OCT_clk_ctrl = 0


def main():
    rospy.init_node('robot_data_logger', anonymous=True)
    path2file = '/home/xihan/catkin_ws/src/robotic_oct/scripts/robot_data_log.csv'
    file_out = open(path2file, 'w')
    writer = csv.writer(file_out)
    freq = 30
    rate = rospy.Rate(freq)
    print('start recording.')
    while not rospy.is_shutdown():
        if T_O_ee is not None:
            data = T_O_ee.flatten()
        if OCT_clk_ctrl == 1:
            # data = np.append(data, rospy.get_time())
            writer.writerow(data)
        rate.sleep()
    print('\nend recording.')
    file_out.close()


if __name__ == "__main__":
    main()
