#!/usr/bin/env python3
import sys
import time
import rospy
import actionlib
import numpy as np
from DoScan import *
from DoTranslationalScan import *
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal


def scan_starts_callback(msg):
  global scan_starts
  scan_starts_raw = msg.data
  num_scans = int(len(scan_starts_raw)/3)
  scan_starts = np.reshape(scan_starts_raw, (num_scans, 3))


def scan_length_callback(msg):
  global scan_length
  scan_length = msg.data    # length of single scan


scan_starts = None
scan_length = None

# initialize ros node
rospy.init_node('rv_panda_test', anonymous=True)
rospy.Subscriber('OCT_scan_path_starts', Float64MultiArray, scan_starts_callback)
rospy.Subscriber('OCT_scan_path_length', Float64, scan_length_callback)
scan_process = DoScan()
# scan_process = DoTranslationalScan()

while scan_length is None or scan_starts is None:
  if rospy.is_shutdown():
    sys.exit(0)
print('scan path received')

# Create a ros action client to communicate with the driver
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()
# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'
start_time = time.time()
for scan in range(len(scan_starts)):
  target.pose.position.x = scan_starts[scan][0] - scan_length
  target.pose.position.y = scan_starts[scan][1]
  target.pose.position.z = 0.18  # scan_starts[scan][2]
  target.pose.orientation.x = 1.00
  target.pose.orientation.y = 0.00
  target.pose.orientation.z = 0.00
  target.pose.orientation.w = 0.00
  print('scan:', scan, 'length[m]:', scan_length)
  print(target.pose.position.x, target.pose.position.y, target.pose.position.z)
  T_O_tar = scan_process.get_target_pose()
  T_O_tar[0, -1] = target.pose.position.x
  T_O_tar[1, -1] = target.pose.position.y
  T_O_tar[2, -1] = target.pose.position.z
  scan_process.set_target_pose(T_O_tar)
  scan_process.set_scan_dist(scan_length*0.8)  # make scan length shorter for safety
  # Create goal from target pose
  goal = MoveToPoseGoal(goal_pose=target)
  # Send goal and wait for it to finish
  client.send_goal(goal)
  client.wait_for_result()
  # do scan process
  scan_process.doScanProcess()
print('scan process took:', (time.time()-start_time)/60, 'minutes')
# go home
target.pose.position.x = 0.35
target.pose.position.y = 0.00
target.pose.position.z = 0.40
target.pose.orientation.x = 1.00
target.pose.orientation.y = 0.00
target.pose.orientation.z = 0.00
target.pose.orientation.w = 0.00
# Create goal from target pose
goal = MoveToPoseGoal(goal_pose=target)
# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
