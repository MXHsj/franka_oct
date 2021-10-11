#! /usr/bin/env python3
'''
motion planner for pure translational OCT scan
this is just a test case
'''
import rospy
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped


class TranslationalScan():
    T_O_ee = None       # T base to eef
    T_O_tar = None      # T base to target
    T_cam_tar = None    # T realsense to target
    isContact = False
    in_plane_rot_err = None
    surf_height_ratio = None      # target surface height
    out_of_plane_slope = None
    OCT_clk_ctrl_msg = Int16()
    vel_msg = Float64MultiArray()
    vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    eef_wrench_msg = WrenchStamped()

    def __init__(self):
        # initialize ROS node
        rospy.init_node('OCT_scan_test', anonymous=True)
        # subscriber
        rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.ee_callback)
        rospy.Subscriber('franka_state_controller/F_ext', WrenchStamped, self.force_callback)
        rospy.Subscriber('OCT_img_fb', Float64MultiArray, self.OCT_img_callback)
        # publisher
        tar_pub = rospy.Publisher('target_pose', Float64MultiArray, queue_size=1)
        op_pub = rospy.Publisher('isContact', Bool, queue_size=1)
        OCT_clk_ctrl_pub = rospy.Publisher('OCT_clk_ctrl', Int16, queue_size=1)
        vel_pub = rospy.Publisher('franka_cmd_vel', Float64MultiArray, queue_size=1)

        while self.T_O_ee is None or self.surf_height_ratio is None:
            pass    # wait for messages are received
        print("robot state received \nconnection establised")

        Fz_d = 1.0 + 1.0 + self.eef_wrench_msg.wrench.force.z
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():  # landing
            # vz = -kp*(desired_surf_height-surf_height_ratio)
            self.vel_msg.data[2] = -0.001*(0.6-self.surf_height_ratio)
            if self.surf_height_ratio >= 0.6:
                print('start scanning')
                break
            vel_pub.publish(self.vel_msg)
            rate.sleep()

        self.OCT_clk_ctrl_msg.data = 1
        in_plane_rot_ierr = 0
        while not rospy.is_shutdown():
            OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
            vel_pub.publish(self.vel_msg)
            x = self.T_O_ee[0, 3]
            z = self.T_O_ee[2, 3]
            # self.vel_msg.data[0] = 0.0001
            self.vel_msg.data[2] = -5e-3*(0.7-self.surf_height_ratio)
            in_plane_rot_ierr += self.in_plane_rot_err
            self.vel_msg.data[3] = 0.002 * self.in_plane_rot_err + 1e-8*in_plane_rot_ierr
            # self.vel_msg.data[4] = math.copysign(1, 0.0001) * 0.015*self.out_of_plane_slope
            if x > 0.46:
                print('finish scan')
                break
            rate.sleep()

        self.vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.OCT_clk_ctrl_msg.data = 0
        OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
        vel_pub.publish(self.vel_msg)

    def isArriveEntry(self):
        self.isContact = True

    def convert2base(self):
        if self.T_O_ee is not None and self.T_cam_tar is not None:
            T_O_cam = np.matmul(self.T_O_ee, self.T_ee_cam)
            T_O_tar = np.matmul(T_O_cam, self.T_cam_tar)
        else:
            T_O_tar = None
        return T_O_tar

    def ee_callback(self, msg):
        EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
        self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                                EE_pos[12:16]]).transpose()

    def force_callback(self, msg):
        self.eef_wrench_msg = msg

    def cam_tar_callback(self, msg):
        cam_tar = list(msg.data)
        # transformation from camera to target
        self.T_cam_tar = \
            np.array([[cam_tar[0], cam_tar[3], cam_tar[6], cam_tar[9]],
                      [cam_tar[1], cam_tar[4], cam_tar[7], cam_tar[10]],
                      [cam_tar[2], cam_tar[5], cam_tar[8], cam_tar[11]],
                      [0.0, 0.0, 0.0, 1.0]])
        # print(T_cam_tar)

    def OCT_img_callback(self, msg):
        self.surf_height_ratio = msg.data[0]
        self.in_plane_rot_err = msg.data[1]
        self.out_of_plane_slope = msg.data[2]


if __name__ == "__main__":
    TranslationalScan()
