#! /usr/bin/env python3
import sys
import rospy
from cv2 import cv2
import open3d as o3d
import selectinwindow
from GetRealSenseData import *

# Set recursion limit
sys.setrecursionlimit(10 ** 9)


def generateWaypoints(rec_obj):
  num_vert_pnts = int(rec_obj.outRect.h*0.08)
  num_hori_pnts = int(rec_obj.outRect.w*0.05)
  vert_pnt_dist = int(rec_obj.outRect.h/(num_vert_pnts+1))
  hori_pnt_dist = int(rec_obj.outRect.w/(num_hori_pnts+1))
  wp = list()
  for row in range(num_vert_pnts):
    for col in range(num_hori_pnts):
      if row % 2 == 0:
        wp.append([rec_obj.outRect.x+hori_pnt_dist*(col+1),
                   rec_obj.outRect.y+vert_pnt_dist*(row+1)])
      else:
        wp.append([rec_obj.outRect.x+hori_pnt_dist*(num_hori_pnts+1) -
                   hori_pnt_dist*(col+1),
                   rec_obj.outRect.y+vert_pnt_dist*(row+1)])
  return wp


def visualizeWaypoints(img, waypoints):
  for i in range(len(waypoints)):
    cv2.circle(img, tuple(waypoints[i]), 2, (0, 255, 0))


# exit
def breakLoop(vis):
  vis.destroy_window()
  cv2.destroyAllWindows()
  sys.exit()


realsense = GetRealSenseData()
cv2.namedWindow('RGB-depth', cv2.WINDOW_AUTOSIZE)
ROI_rec = selectinwindow.DragRectangle(realsense.color_image, 'RGB-depth', 640, 480)
cv2.setMouseCallback(ROI_rec.wname, selectinwindow.dragrect, ROI_rec)
wp = None
# open3d stuff
pointcloud = o3d.geometry.PointCloud()
outlier_cloud = o3d.geometry.PointCloud()

cam_intr = o3d.camera.PinholeCameraIntrinsic(
    realsense.intr.width, realsense.intr.height,
    realsense.intr.fx, realsense.intr.fy,
    realsense.intr.ppx, realsense.intr.ppy)
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window("pointcloud", 640, 480)
vis.register_key_callback(ord("Q"), breakLoop)
isGeometryAdded = False
isUpdateStream = True

try:
  while True:
    key = cv2.waitKey(33)
    if key & 0xFF == ord('q') or key == 27:
      print("quit")
      breakLoop(vis)
      break
    elif key == ord('\r'):
      print("Dragged rectangle coordinates")
      print(ROI_rec.outRect.x, ROI_rec.outRect.y,
            ROI_rec.outRect.w, ROI_rec.outRect.h)
      wp = generateWaypoints(ROI_rec)
      print("waypoints: ", len(wp))
    elif key == ord('c'):
      print("clear ROI selection")
      ROI_rec.resetRec()
    elif key == ord('f'):
      isUpdateStream = not isUpdateStream
      if not isUpdateStream:
        print("stop streaming")
      else:
        isGeometryAdded = False
        print("resume streaming")
    if isUpdateStream:
      # stream color & depth aligned frames
      realsense.stream_depth2color_aligned()
      cv2.imshow('RGB-depth', realsense.color_image)
      if wp is not None:
        visualizeWaypoints(realsense.color_image, wp)

      # ROI selection in color frame
      ROI_rec.updateImg(realsense.color_image)
      selectinwindow.clearCanvasNDraw(ROI_rec)

    if not isUpdateStream:
      if not isGeometryAdded:
        # create open3d pointcloud
        vis.remove_geometry(pointcloud)
        depth = o3d.geometry.Image(realsense.depth_image)
        rgb = cv2.cvtColor(realsense.color_image, cv2.COLOR_RGB2BGR)
        color = o3d.geometry.Image(rgb)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intr, project_valid_depth_only=True)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pointcloud = pcd
        del pcd
        # pointcloud.uniform_down_sample(every_k_points=100)

        # segment plane
        plane_model, inliers = pointcloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=100)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        outlier_cloud = pointcloud.select_by_index(inliers, invert=True)
        outlier_cloud.paint_uniform_color([91/255, 94/255, 186/255])

        # estimate surface normal (search range 0.01m, search neighbors 10)
        outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=10))
        # coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=pointcloud.get_center())

        # o3d visualization
        vis.add_geometry(pointcloud)
        # vis.add_geometry(coord)
        vis.add_geometry(outlier_cloud)
        isGeometryAdded = True
      # vis.update_geometry(pointcloud)
      # vis.update_geometry(coord)
      # vis.update_geometry(outlier_cloud)
    vis.poll_events()
    vis.update_renderer()

finally:
  realsense.stop_stream()
