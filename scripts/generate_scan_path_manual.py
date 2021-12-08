#! /usr/bin/env python3
import sys
import rospy
import numpy as np
from cv2 import cv2
import selectinwindow
from GetRealSenseData import *
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

# Set recursion limit
sys.setrecursionlimit(10 ** 9)


def convert2base(P_cam):
  '''
  convert target from camera frame to base frame
  '''
  global T_O_ee, T_ee_cam
  P_base = P_cam.copy()
  for i in range(len(P_base)):
    curr_P_cam = np.append(P_cam[i, :], 1)
    T_O_cam = np.matmul(T_O_ee, T_ee_cam)
    curr_P_base = np.matmul(T_O_cam, curr_P_cam)
    P_base[i, :] = curr_P_base[:3].transpose()
  return P_base


def generate_scan_path(rec_obj, rs_obj):
  corner_pix = [[rec_obj.outRect.x,                   rec_obj.outRect.y],
                [rec_obj.outRect.x+rec_obj.outRect.w, rec_obj.outRect.y],
                [rec_obj.outRect.x,                   rec_obj.outRect.y+rec_obj.outRect.h],
                [rec_obj.outRect.x+rec_obj.outRect.w, rec_obj.outRect.y+rec_obj.outRect.h]]
  corner_xyz = rs_obj.get_xyz(corner_pix)
  scan_width = corner_xyz[1][0] - corner_xyz[0][0]    # (x+w,y) - (x,y)
  scan_length = corner_xyz[2][1] - corner_xyz[0][1]   # (x,y+h) - (x,y)
  print("ROI width[mm] ", scan_width*1000, "ROI height[mm] ", scan_length*1000)
  num_scans = int(np.ceil(scan_width*1000/(7.8-0)))   # 7.8 mm lateral FOV; 0 mm overlap
  scan_interv = scan_width/num_scans
  scan_starts_cam = np.zeros((num_scans, 3))
  for i in range(num_scans):
    scan_starts_cam[i, :] = corner_xyz[0]
    scan_starts_cam[i, 0] += i*scan_interv
    scan_starts_cam[i, 2] = np.mean(corner_xyz[:, -1])-0.12   # [m] safe distance
  scan_starts_base = convert2base(scan_starts_cam)  # convert to base frame
  print(scan_starts_base)
  return scan_starts_base, scan_length


def visualize_scan_path(img, scan_starts, scan_length):
  pass


def break_loop():
  cv2.destroyAllWindows()
  sys.exit()


def ee_callback(msg):
  global T_O_ee
  EE_pos = msg.O_T_EE   # inv 4x4 matrix
  T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


# home pose
T_O_ee = \
    np.array([[1.0, 0.0, 0.0, 0.0],
              [0.0, -1.0, 0.0, 0.0],
              [0.0, 0.0, -1.0, 0.0],
              [0.35, 0.0, 0.35, 1.0]]).transpose()
# hand-eye config
T_ee_cam = \
    np.array([[0.0, 1.0, 0.0, 0.0],
              [-1.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 1.0, 0.0],
              [-0.112, -0.0340, -0.065, 1.0]]).transpose()

realsense = GetRealSenseData()
cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
ROI_rec = selectinwindow.DragRectangle(realsense.color_image, 'realsense', 640, 480)
cv2.setMouseCallback(ROI_rec.wname, selectinwindow.dragrect, ROI_rec)
isPublishPath = False
scan_starts = None
scan_length = None

rospy.init_node('OCT_scan_ROI_manual', anonymous=True)
rospy.Subscriber('franka_state_controller/franka_states', FrankaState, ee_callback)
scan_starts_pub = rospy.Publisher('OCT_scan_path_starts', Float64MultiArray, queue_size=1)
scan_length_pub = rospy.Publisher('OCT_scan_path_length', Float64, queue_size=1)
scan_starts_msg = Float64MultiArray()
scan_length_msg = Float64()
rate = rospy.Rate(20)

try:
  while not rospy.is_shutdown():
    realsense.stream_depth2color_aligned()
    ROI_rec.updateImg(realsense.color_image)
    selectinwindow.clearCanvasNDraw(ROI_rec)
    if scan_starts is not None and scan_length is not None:
      visualize_scan_path(realsense.color_image, scan_starts, scan_length)
    cv2.imshow('realsense', realsense.color_image)
    if isPublishPath:
      scan_starts_msg.data = scan_starts.flatten()
      scan_length_msg.data = scan_length
      scan_starts_pub.publish(scan_starts_msg)
      scan_length_pub.publish(scan_length_msg)
    # ==================== keyboard command ====================
    key = cv2.waitKey(33)
    if key & 0xFF == ord('q') or key == 27:
      print("quit")
      break_loop()
    elif key == ord('\r'):
      print("Dragged rectangle coordinates")
      scan_starts, scan_length = generate_scan_path(ROI_rec, realsense)
      print("number of scan path: ", len(scan_starts))
    elif key == ord('p'):
      isPublishPath = not isPublishPath
      print("publish scan path: ", isPublishPath)
    elif key == ord('c'):
      print("clear ROI selection")
      ROI_rec.resetRec()
    # ==========================================================
    rate.sleep()
finally:
  pass
