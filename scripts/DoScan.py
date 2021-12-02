#! /usr/bin/env python3
'''
motion planner for pure translational OCT scan
this is just a test case
'''
import rospy
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import TwistStamped


class DoScan():
  T_O_ee = None       # T base to eef
  in_plane_rot_err = None
  last_in_plane_rot_err = None
  surf_height_ratio = None      # target surface height
  out_of_plane_slope = None
  OCT_clk_ctrl_msg = Int8()
  # vel_msg = Float64MultiArray()
  # vel_msg.data = [0.0]*6
  vel_msg = TwistStamped()
  vel_msg.twist.linear.x = 0.0
  vel_msg.twist.linear.y = 0.0
  vel_msg.twist.linear.z = 0.0
  vel_msg.twist.angular.x = 0.0
  vel_msg.twist.angular.y = 0.0
  vel_msg.twist.angular.z = 0.0
  last_vel_msg = vel_msg
  pos_msg = Float64MultiArray()
  pos_msg.data = [0.0]*12

  def __init__(self):
    # initialize ROS node
    # rospy.init_node('OCT_scan_process', anonymous=True)
    # subscriber
    rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.ee_callback)
    rospy.Subscriber('OCT_img_fb', Float64MultiArray, self.OCT_img_callback)
    # publisher
    self.OCT_clk_ctrl_pub = rospy.Publisher('OCT_clk_ctrl', Int8, queue_size=50)
    self.vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    self.pos_pub = rospy.Publisher('franka_cmd_pos', Float64MultiArray, queue_size=1)
    # define initial pose
    y_offset = -(7.5-1.0)*1e-3    # 7.5(BScan width) - 1.0(overlap) [mm]
    x_offset = 0.01               # [mm]
    self.T_O_tar = \
        np.array([[1.0, 0.0, 0.0, 0.434 + 0*x_offset],
                  [0.0, -1.0, 0.0, 0.0 + 6*y_offset],
                  [0.0, 0.0, -1.0, 0.13],
                  [0.0, 0.0, 0.0, 1]])
    self.rate = rospy.Rate(1000)
    self.scan_dist = 0.045  # 0.113          # scan distance [m]
    self.lin_vel_x = 0.0008       # [m/s] scan along +x direction
    print("connecting to OCT desktop ...")
    while self.T_O_ee is None or self.surf_height_ratio is None:
      if rospy.is_shutdown():
        return
    print("robot state received \nconnection establised")

  def doScanProcess(self):
    # ---------- go to entry pose ----------
    while not rospy.is_shutdown():
      self.pos_msg.data = self.T_O_tar[:3, :4].transpose().flatten()
      T_error = np.subtract(self.T_O_tar, self.T_O_ee)
      trans_error = T_error[0:3, 3]
      rot_error = T_error[0:3, 0:3].flatten()
      isReachedTrans = True if sum([abs(err) < 0.0005 for err in trans_error]) == len(trans_error) else False
      isReachedRot = True if sum([abs(err) < 0.05 for err in rot_error]) == len(rot_error) else False
      print(isReachedTrans, isReachedRot)
      if isReachedRot and isReachedTrans:
        print('reached entry pose')
        break
      self.pos_pub.publish(self.pos_msg)
      self.rate.sleep()
    # ---------- landing ----------
    print('landing ...')
    while not rospy.is_shutdown():
      self.vel_msg.twist.linear.z = \
          0.125*-0.005*(0.7-self.surf_height_ratio) + (1-0.125)*self.last_vel_msg.twist.linear.z
      # self.vel_msg.twist.angular.x = \
      #     0.125*-0.020*self.in_plane_rot_err + (1-0.125)*self.last_vel_msg.twist.angular.x
      # if self.surf_height_ratio >= 0.7 and abs(self.in_plane_rot_err) < 0.08:
      if self.surf_height_ratio >= 0.7:
        break
      self.vel_pub.publish(self.vel_msg)
      self.rate.sleep()
    # ---------- scan ----------
    print('start scanning ...')
    self.OCT_clk_ctrl_msg.data = 1
    while not rospy.is_shutdown():
      self.vel_msg.twist.linear.x = self.lin_vel_x
      self.vel_msg.twist.linear.z = \
          0.125*-0.005*(0.7-self.surf_height_ratio) + (1-0.125)*self.last_vel_msg.twist.linear.z
      # self.vel_msg.twist.angular.x = \
      #     0.125*-0.020*self.in_plane_rot_err + (1-0.125)*self.last_vel_msg.twist.angular.x
      self.last_vel_msg = self.vel_msg
      self.OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
      self.vel_pub.publish(self.vel_msg)
      if self.T_O_ee[0, 3] >= self.T_O_tar[0, 3] + self.scan_dist:  # scan along x direction
        break
      self.rate.sleep()
    # ---------- let OCT desktop save data & clean up ----------
    # TODO: send signal to OCT desktop for data saving
    print('finish scan')
    self.finish()
    rospy.on_shutdown(self.finish)

  def get_target_pose(self) -> np.ndarray:
    return self.T_O_tar

  def set_target_pose(self, T_O_tar_new: np.ndarray):
    """
    overwrite T_O_tar
    """
    self.T_O_tar = T_O_tar_new

  def set_scan_dist(self, scan_dist_new: float):
    """
    overwrite scan_dist
    """
    self.scan_dist = scan_dist_new

  def finish(self):
    self.OCT_clk_ctrl_msg.data = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.angular.x = 0
    self.vel_msg.twist.angular.y = 0
    self.vel_msg.twist.angular.z = 0
    for i in range(3000):
      self.OCT_clk_ctrl_pub.publish(self.OCT_clk_ctrl_msg)
      self.vel_pub.publish(self.vel_msg)

  def ee_callback(self, msg):
    EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()

  def OCT_img_callback(self, msg):
    self.surf_height_ratio = msg.data[0]
    self.in_plane_rot_err = msg.data[1]
    self.out_of_plane_slope = msg.data[2]


if __name__ == "__main__":
  scan = DoScan()
  scan.doScanProcess()
