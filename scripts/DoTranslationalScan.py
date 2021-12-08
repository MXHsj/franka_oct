#! /usr/bin/env python3
'''
do single round OCT scan (translational motion only)
'''
import math
import rospy
import actionlib
import numpy as np
from std_msgs.msg import Int8
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal


class DoTranslationalScan():
  T_O_ee = None
  in_plane_rot_err = None
  surf_height_ratio = None        # target surface height
  isRemoteDataSaved = 0           # 1: data saved on OCT desktop; 0: data hasn't been saved
  scan_flag_msg = Int8()          # 1: scanning; 0: scan stop;
  vel_msg = TwistStamped()
  vel_msg.twist.linear.x = 0.0
  vel_msg.twist.linear.y = 0.0
  vel_msg.twist.linear.z = 0.0
  vel_msg.twist.angular.x = 0.0
  vel_msg.twist.angular.y = 0.0
  vel_msg.twist.angular.z = 0.0
  vel_msg_last = vel_msg

  def __init__(self):
    # subscriber & publisher
    rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.ee_callback)
    rospy.Subscriber('OCT_remote_response', Float64MultiArray, self.OCT_remote_callback)
    self.scan_flag_pub = rospy.Publisher('OCT_scan_flag', Int8, queue_size=50)
    self.vel_pub = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
    # entry pose
    self.T_O_tar = \
        np.array([[1.0, 0.0, 0.0, 0.430],
                  [0.0, -1.0, 0.0, 0.00],
                  [0.0, 0.0, -1.0, 0.20],
                  [0.0, 0.0, 0.0, 1.0]])
    self.rate = rospy.Rate(1000)
    self.scan_dist = 0.043        # [m] scan distance
    self.lin_vel_x = 0.00060      # [m/s] scan velocity +x direction
    print("connecting to OCT desktop ...")
    while self.T_O_ee is None or self.surf_height_ratio is None:
      if rospy.is_shutdown():
        return
    print("robot state received \nconnection establised")

  def doScanProcess(self):
    # ---------- landing ----------
    print('landing ...')
    while not rospy.is_shutdown():
      self.vel_msg.twist.linear.z = -0.0035*(0.7-self.surf_height_ratio)
      vel_msg_filtered = self.IIR_filter()
      self.vel_msg_last = vel_msg_filtered
      self.vel_pub.publish(vel_msg_filtered)
      if self.surf_height_ratio > 0.7:
        break
      self.rate.sleep()
    # ---------- scan ----------
    print('scanning ...')
    self.scan_flag_msg.data = 1
    while not rospy.is_shutdown():
      self.vel_msg.twist.linear.x = self.lin_vel_x
      self.vel_msg.twist.linear.z = -0.0035*(0.7-self.surf_height_ratio)
      vel_msg_filtered = self.IIR_filter()
      self.vel_msg_last = vel_msg_filtered
      self.scan_flag_pub.publish(self.scan_flag_msg)
      self.vel_pub.publish(vel_msg_filtered)
      if self.T_O_ee[0, 3] >= self.T_O_tar[0, 3] + self.scan_dist:  # scan along x direction
        break
      self.rate.sleep()
    # ---------- let OCT desktop save data & clean up ----------
    # TODO: send signal to OCT desktop for data saving
    self.finish()
    print('finish scan, waiting for remote data to be saved ...')
    while not rospy.is_shutdown():
      if self.isRemoteDataSaved == 1:
        print('data saved on OCT desktop')
        self.isRemoteDataSaved = 0
        break
      self.rate.sleep()

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
    self.scan_flag_msg.data = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.linear.x = 0
    self.vel_msg.twist.angular.x = 0
    self.vel_msg.twist.angular.y = 0
    self.vel_msg.twist.angular.z = 0
    for i in range(3000):
      self.scan_flag_pub.publish(self.scan_flag_msg)
      self.vel_pub.publish(self.vel_msg)

  def IIR_filter(self) -> TwistStamped:
    """
    apply IIR filter on velocity commands
    """
    p = 0.125
    filtered = TwistStamped()
    filtered.twist.linear.x = self.vel_msg.twist.linear.x
    filtered.twist.linear.y = p*self.vel_msg.twist.linear.y + (1-p)*self.vel_msg_last.twist.linear.y
    filtered.twist.linear.z = p*self.vel_msg.twist.linear.z + (1-p)*self.vel_msg_last.twist.linear.z
    filtered.twist.angular.x = p*self.vel_msg.twist.angular.x + (1-p)*self.vel_msg_last.twist.angular.x
    filtered.twist.angular.y = p*self.vel_msg.twist.angular.y + (1-p)*self.vel_msg_last.twist.angular.y
    filtered.twist.angular.z = p*self.vel_msg.twist.angular.z + (1-p)*self.vel_msg_last.twist.angular.z
    return filtered

  def ee_callback(self, msg):
    EE_pos = msg.O_T_EE   # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()

  def OCT_remote_callback(self, msg):
    self.surf_height_ratio = msg.data[0]
    self.in_plane_rot_err = msg.data[1]
    self.isRemoteDataSaved = msg.data[2]


if __name__ == "__main__":
  rospy.init_node('OCT_translational_scan_process', anonymous=True)
  scan = DoTranslationalScan()
  # go to entry pose
  client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
  client.wait_for_server()
  target = PoseStamped()
  target.header.frame_id = 'panda_link0'
  # define entry pose & scan length
  overlap = 0.0                     # [mm] overlap
  y_offset = -(7.8-overlap)*1e-3    # 7.8(BScan width) [mm] - (overlap) [mm]
  target.pose.position.x = scan.T_O_tar[0, -1]
  target.pose.position.y = scan.T_O_tar[1, -1] + 0*y_offset
  target.pose.position.z = scan.T_O_tar[2, -1]
  target.pose.orientation.x = 1.00
  target.pose.orientation.y = 0.00
  target.pose.orientation.z = 0.00
  target.pose.orientation.w = 0.00
  scan.T_O_tar[0, -1] = target.pose.position.x
  scan.T_O_tar[1, -1] = target.pose.position.y
  scan.T_O_tar[2, -1] = target.pose.position.z
  goal = MoveToPoseGoal(goal_pose=target)
  # Send goal and wait for it to finish
  client.send_goal(goal)
  client.wait_for_result()
  # translational scan motion
  scan.doScanProcess()
  rospy.on_shutdown(scan.finish)
