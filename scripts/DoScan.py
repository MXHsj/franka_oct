#! /usr/bin/env python3
'''
do single round OCT scan (with in-plane rotation)
'''
import math
import rospy
import numpy as np
from std_msgs.msg import Int8
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class DoScan():
  pureTranslation = False
  T_O_ee = None
  in_plane_rot_err = None
  surf_height_ratio = None        # target surface height
  isRemoteDataSaved = 0           # 1: data saved on OCT desktop; 0: data hasn't been saved
  scan_flag_msg = Int8()          # 1: scanning; 0: scan stop;
  vel_msg = Twist()
  vel_msg.linear.x = 0.0
  vel_msg.linear.y = 0.0
  vel_msg.linear.z = 0.0
  vel_msg.angular.x = 0.0
  vel_msg.angular.y = 0.0
  vel_msg.angular.z = 0.0
  vel_msg_last = vel_msg
  pos_msg = Float64MultiArray()
  pos_msg.data = [0.0]*12

  def __init__(self):
    # subscriber & publisher
    rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.ee_callback)
    rospy.Subscriber('OCT_remote_response', Float64MultiArray, self.OCT_remote_callback)
    self.scan_flag_pub = rospy.Publisher('OCT_scan_flag', Int8, queue_size=1)
    self.vel_pub = rospy.Publisher('franka_cmd_acc', Twist, queue_size=1)
    self.pos_pub = rospy.Publisher('franka_cmd_pos', Float64MultiArray, queue_size=1)
    # entry pose
    self.T_O_tar = \
        np.array([[1.0, 0.0, 0.0, 0.430],
                  [0.0, -1.0, 0.0, 0.00],
                  [0.0, 0.0, -1.0, 0.20],
                  [0.0, 0.0, 0.0, 1.0]])
    self.rate = rospy.Rate(1000)
    self.scan_dist = 0.040        # [m] scan distance
    self.lin_vel_x = 0.00060      # [m/s] scan velocity +x direction
    self.surf_height_d = 0.70      # desired surface height ratio
    print("connecting to OCT desktop ...")
    while self.T_O_ee is None or self.surf_height_ratio is None:
      if rospy.is_shutdown():
        return
    print("robot state received \nconnection establised")

  def go_to_entry(self):
    # ---------- go to entry pose ----------
    while not rospy.is_shutdown():
      self.pos_msg.data = self.T_O_tar[:3, :4].transpose().flatten()
      T_error = np.subtract(self.T_O_tar, self.T_O_ee)
      trans_error = T_error[0:3, 3]
      rot_error = T_error[0:3, 0:3].flatten()
      isReachedTrans = True if sum([abs(err) < 0.0018 for err in trans_error]) == len(trans_error) else False
      isReachedRot = True if sum([abs(err) < 0.03 for err in rot_error]) == len(rot_error) else False
      # print(isReachedTrans, isReachedRot)
      if isReachedRot and isReachedTrans:
        print('reached entry pose')
        return
      self.pos_pub.publish(self.pos_msg)
      self.rate.sleep()

  def doScanProcess(self):
    # ---------- landing ----------
    print('landing ...')
    while not rospy.is_shutdown():
      # linear velocity along approach vector
      # self.vel_msg.linear.y = math.cos(math.atan2(self.T_O_ee[2, -1], self.T_O_ee[1, -1]))*desired_vel
      # self.vel_msg.linear.z = math.sin(math.atan2(self.T_O_ee[2, -1], self.T_O_ee[1, -1]))*desired_vel
      self.vel_msg.linear.z = -0.0016*np.tanh([self.surf_height_d-self.surf_height_ratio])
      if not self.pureTranslation:
        self.vel_msg.angular.x = -0.020*np.tanh([self.in_plane_rot_err])
        # pass
      vel_msg_filtered = self.IIR_filter()
      self.vel_pub.publish(vel_msg_filtered)
      self.vel_msg_last = vel_msg_filtered
      if self.pureTranslation:
        if self.surf_height_ratio > self.surf_height_d:
          break
      else:
        if self.surf_height_ratio > self.surf_height_d and abs(self.in_plane_rot_err) < 0.05:
          break
      self.rate.sleep()
    # ---------- scan ----------
    print('scanning ...')
    self.scan_flag_msg.data = 1
    while not rospy.is_shutdown():
      self.vel_msg.linear.x = self.lin_vel_x
      # linear velocity along approach vector
      # self.vel_msg.linear.y = math.cos(math.atan2(self.T_O_ee[2, -1], self.T_O_ee[1, -1]))*desired_vel
      # self.vel_msg.linear.z = math.sin(math.atan2(self.T_O_ee[2, -1], self.T_O_ee[1, -1]))*desired_vel
      self.vel_msg.linear.z = -0.0030*np.tanh([self.surf_height_d-self.surf_height_ratio])
      if not self.pureTranslation:
        self.vel_msg.angular.x = -0.020*np.tanh([self.in_plane_rot_err])
        # pass
      vel_msg_filtered = self.IIR_filter()
      self.vel_msg_last = vel_msg_filtered
      self.vel_pub.publish(vel_msg_filtered)
      self.scan_flag_pub.publish(self.scan_flag_msg)
      if self.T_O_ee[0, 3] >= self.T_O_tar[0, 3] + self.scan_dist:  # scan along x direction
        break
      self.rate.sleep()
    # ---------- let OCT desktop save data & clean up ----------
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
    self.vel_msg.linear.x = 0
    self.vel_msg.linear.x = 0
    self.vel_msg.linear.x = 0
    self.vel_msg.angular.x = 0
    self.vel_msg.angular.y = 0
    self.vel_msg.angular.z = 0
    for i in range(6000):
      self.scan_flag_pub.publish(self.scan_flag_msg)
      self.vel_pub.publish(self.vel_msg)

  def IIR_filter(self) -> Twist:
    """
    apply IIR filter on velocity commands
    """
    p = 0.0667
    filtered = Twist()
    filtered.linear.x = self.vel_msg.linear.x
    filtered.linear.y = p*self.vel_msg.linear.y + (1-p)*self.vel_msg_last.linear.y
    filtered.linear.z = p*self.vel_msg.linear.z + (1-p)*self.vel_msg_last.linear.z
    filtered.angular.x = p*self.vel_msg.angular.x + (1-p)*self.vel_msg_last.angular.x
    filtered.angular.y = p*self.vel_msg.angular.y + (1-p)*self.vel_msg_last.angular.y
    filtered.angular.z = p*self.vel_msg.angular.z + (1-p)*self.vel_msg_last.angular.z
    return filtered

  def ee_callback(self, msg):
    EE_pos = msg.O_T_EE   # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()

  def OCT_remote_callback(self, msg):
    self.surf_height_ratio = msg.data[0]
    self.in_plane_rot_err = msg.data[1]
    self.isRemoteDataSaved = msg.data[2]


if __name__ == "__main__":
  rospy.init_node('OCT_scan_process', anonymous=True)
  scan = DoScan()
  # ===== define entry pose & scan length =====
  overlap = 0.0                     # [mm] overlap
  y_offset = -(7.8-overlap)*1e-3    # 7.8(BScan width) [mm] - (overlap) [mm]
  # scan.T_O_tar[0, -1] = scan.T_O_tar[0, -1]
  scan.T_O_tar[1, -1] += 1*y_offset
  scan.T_O_tar[2, -1] = 0.17
  scan.scan_dist = 0.040
  # ===== go to entry pose =====
  scan.go_to_entry()
  # ===== translational scan motion =====
  scan.doScanProcess()
  rospy.on_shutdown(scan.finish)
