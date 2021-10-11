#! /usr/bin/env python3
'''
calibrate OCT probe orientation
'''
import csv
import rospy
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray


class TranslationalScan():
    T_O_ee = None       # T base to eef
    T_O_tar = None      # T base to target
    T_cam_tar = None    # T realsense to target
    isContact = False
    in_plane_rot_err = None
    last_in_plane_rot_err = None
    surf_height_ratio = None      # target surface height
    out_of_plane_slope = None
    OCT_clk_ctrl_msg = Int16()
    vel_msg = Float64MultiArray()
    pos_msg = Float64MultiArray()
    vel_msg.data = [0.0]*6
    pos_msg.data = [0.0]*12

    def __init__(self):
        # initialize ROS node
        rospy.init_node('OCT_scan_test', anonymous=True)
        # subscriber
        rospy.Subscriber("franka_state_controller/franka_states",
                         FrankaState, self.ee_callback)
        rospy.Subscriber('OCT_img_fb', Float64MultiArray,
                         self.OCT_img_callback)
        # publisher
        OCT_clk_ctrl_pub = rospy.Publisher(
            'OCT_clk_ctrl', Int16, queue_size=50)
        vel_pub = rospy.Publisher(
            'franka_cmd_vel', Float64MultiArray, queue_size=1)
        pos_pub = rospy.Publisher(
            'franka_cmd_pos', Float64MultiArray, queue_size=1)

        print("connecting to OCT desktop ...")
        while self.T_O_ee is None or self.surf_height_ratio is None:
            pass    # wait for messages are received
        print("robot state received \nconnection establised")

        rate = rospy.Rate(1000)
        T_O_tar = np.array([[1.0, 0.0, 0.0, 0.45],
                            [0.0, -1.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0, 0.075],
                            [0.0, 0.0, 0.0, 1.0]])
        while not rospy.is_shutdown():  # go to entry pose
            self.pos_msg.data = T_O_tar[:3, :4].transpose().flatten()
            T_error = np.subtract(T_O_tar, self.T_O_ee)
            trans_error = T_error[0:3, 3]
            rot_error = T_error[0:3, 0:3].flatten()
            isReachedTrans = True if sum(
                [abs(err) < 0.0005 for err in trans_error]) == len(trans_error) else False
            isReachedRot = True if sum(
                [abs(err) < 0.1 for err in rot_error]) == len(rot_error) else False
            if isReachedRot and isReachedTrans:
                print('reached entry pose')
                break
            pos_pub.publish(self.pos_msg)
            rate.sleep()

        while not rospy.is_shutdown():  # landing
            # vz = -kp*(desired_surf_height-surf_height_ratio)
            self.vel_msg.data[2] = -0.001*(0.65-self.surf_height_ratio)
            if self.surf_height_ratio >= 0.65:
                print('start scanning')
                break
            vel_pub.publish(self.vel_msg)
            rate.sleep()

        self.OCT_clk_ctrl_msg.data = 1
        while not rospy.is_shutdown():  # scan
            OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
            vel_pub.publish(self.vel_msg)
            self.vel_msg.data[2] = -0.001*(0.65-self.surf_height_ratio)
            self.vel_msg.data[3] = 0.006*self.in_plane_rot_err
            rate.sleep()

        print('finish scan')
        self.vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.OCT_clk_ctrl_msg.data = 0
        OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
        vel_pub.publish(self.vel_msg)

    def isArriveEntry(self):
        self.isContact = True
        pass

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
