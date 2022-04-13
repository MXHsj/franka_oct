#!/usr/bin/env python3
'''

'''
import sys
import time
import rospy
import numpy as np
from DoScan import *
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


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
rospy.init_node('OCT_automatic_scan', anonymous=True)
rospy.Subscriber('OCT_scan_path_starts', Float64MultiArray, scan_starts_callback)
rospy.Subscriber('OCT_scan_path_length', Float64, scan_length_callback)
scan_process = DoScan()
# scan_process.pureTranslation = True

while scan_length is None or scan_starts is None:
  if rospy.is_shutdown():
    sys.exit(0)
print('scan path received')

start_time = time.time()
for scan in range(len(scan_starts)):
  # define entry
  scan_process.T_O_tar[0, -1] = scan_starts[scan][0] - scan_length
  scan_process.T_O_tar[1, -1] = scan_starts[scan][1]
  scan_process.T_O_tar[2, -1] = 0.14  # 0.175 scan_starts[scan][2]
  print('next start: x, y, z [m]',
        scan_process.T_O_tar[0, -1], scan_process.T_O_tar[1, -1], scan_process.T_O_tar[2, -1])
  # go to entry
  scan_process.go_to_entry()
  print('scan:', scan, 'length[m]:', scan_length)
  scan_process.set_scan_dist(scan_length*0.9)  # make scan length shorter for safety
  # do scan process
  scan_process.doScanProcess()
print('scan process took:', (time.time()-start_time)/60, 'minutes')
# go home
scan_process.T_O_tar[0, -1] = 0.35
scan_process.T_O_tar[1, -1] = 0.00
scan_process.T_O_tar[2, -1] = 0.35
scan_process.go_to_entry()
