#! /usr/bin/env python3
import open3d as o3d
from GetRealSenseData import *


class GetO3DCloud():
  pass


# exit
def breakLoop(vis):
  vis.destroy_window()
  cv2.destroyAllWindows()
  sys.exit()


if __name__ == "__main__":
  get_realsense_data = GetRealSenseData()

  pointcloud = o3d.geometry.PointCloud()
  cam_intr = o3d.camera.PinholeCameraIntrinsic(
      get_realsense_data.intr.width, get_realsense_data.intr.height,
      get_realsense_data.intr.fx, get_realsense_data.intr.fy,
      get_realsense_data.intr.ppx, get_realsense_data.intr.ppy)
  vis = o3d.visualization.VisualizerWithKeyCallback()
  vis.create_window("pointcloud", 640, 480)
  vis.register_key_callback(ord("Q"), breakLoop)
  isGeometryAdded = False

  cv2.namedWindow('RGB-depth', cv2.WINDOW_AUTOSIZE)
  while True:
    get_realsense_data.stream_depth2color_aligned()
    color_depth_stack = np.vstack((get_realsense_data.color_image, get_realsense_data.depth_colormap))
    cv2.imshow('RGB-depth', color_depth_stack)
    key = cv2.waitKey(10)
    if key & 0xFF == ord('Q') or key == 27:
      breakLoop(vis)
      break

    pointcloud.clear()
    depth = o3d.geometry.Image(get_realsense_data.depth_image)
    rgb = cv2.cvtColor(get_realsense_data.color_image, cv2.COLOR_RGB2BGR)
    color = o3d.geometry.Image(rgb)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam_intr)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pointcloud += pcd
    if not isGeometryAdded:
      vis.add_geometry(pointcloud)
      isGeometryAdded = True
    # update geometry
    vis.update_geometry(pointcloud)
    vis.poll_events()
    vis.update_renderer()
