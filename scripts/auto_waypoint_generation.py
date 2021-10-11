#! /usr/bin/env python3
import sys
import rospy
import numpy as np
from cv2 import cv2
import numpy as np
import selectinwindow
from std_msgs.msg import Float32MultiArray
from pyrealsense2 import pyrealsense2 as rs

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


def getPoint(depth_frame, waypoints):
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    points = []
    for i in range(len(waypoints)):
        pix = waypoints[i]
        # print(pix)
        try:
            # deprojection
            depth_in_met = depth_frame.as_depth_frame().get_distance(pix[1], pix[0])
            pnt = rs.rs2_deproject_pixel_to_point(depth_intrin, pix, depth_in_met)
        except Exception as err:
            print(err)
            pnt = [0.0, 0.0, 0.0]
        points.append(pnt)
    print(points)
    # points_formatted = np.reshape(points, [3, len(waypoints)]).T
    points_formatted = np.array(points).flatten()
    return points_formatted


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
align = rs.align(rs.stream.color)

# depth filter
hole_filling = rs.hole_filling_filter()
spat_filter = rs.spatial_filter()       # reduce temporal noise
spat_filter.set_option(rs.option.filter_smooth_alpha, 1)
spat_filter.set_option(rs.option.filter_smooth_delta, 50)

cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
color_image = np.ones([480, 640, 3], dtype=np.uint8)
ROI_rec = selectinwindow.DragRectangle(
    color_image, 'RealSense', 640, 480)
cv2.setMouseCallback(ROI_rec.wname, selectinwindow.dragrect, ROI_rec)
wp = None

waypoints_pub = rospy.Publisher(
    'generated_waypoints3D', Float32MultiArray, queue_size=1)
waypoints_msg = Float32MultiArray()

rospy.init_node('auto_waypoint_generation', anonymous=True)
try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # align depth to color frame
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # apply depth filters
        filtered = spat_filter.process(depth_frame)
        filtered = hole_filling.process(filtered)

        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filtered.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # ROI selection
        ROI_rec.updateImg(color_image)
        selectinwindow.clearCanvasNDraw(ROI_rec)
        key = cv2.waitKey(33)
        if key & 0xFF == ord('q') or key == 27:
            print('quit')
            break
        elif key == ord('\r'):
            print("Dragged rectangle coordinates")
            print(ROI_rec.outRect.x, ROI_rec.outRect.y,
                  ROI_rec.outRect.w, ROI_rec.outRect.h)
            wp = generateWaypoints(ROI_rec)
            pnts = getPoint(depth_frame, wp)
            print("points: ", len(pnts))
            print("waypoints: ", len(wp))
            # print(pnts)
            # print(wp)
        elif key == ord('c'):
            print("clear ROI selection")
            ROI_rec.resetRec()
            wp = None
        elif key == ord('p'):
            print("publish sampled waypoints")
            waypoints_msg.data = pnts

        if wp is not None:
            visualizeWaypoints(color_image, wp)

        # Show images
        cv2.imshow('RealSense', color_image)
        waypoints_pub.publish(waypoints_msg)
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
