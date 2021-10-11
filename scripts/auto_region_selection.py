#! /usr/bin/env python3
import sys
import numpy as np
from cv2 import cv2
import numpy as np
import selectinwindow
from pyrealsense2 import pyrealsense2 as rs

# Set recursion limit
sys.setrecursionlimit(10 ** 9)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
# profile = pipeline.start(config)
# depth_sensor = profile.get_device().first_depth_sensor()
# depth_scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)
pipeline.start(config)

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

        ROI_rec.updateImg(color_image)

        key = cv2.waitKey(33)
        if key & 0xFF == ord('q') or key == 27:
            print('quit')
            break
        elif key == ord('\r'):
            print("Dragged rectangle coordinates")
            print(str(ROI_rec.outRect.x) + ',' + str(ROI_rec.outRect.y) + ',' +
                  str(ROI_rec.outRect.w) + ',' + str(ROI_rec.outRect.h))
            ROI_rec = selectinwindow.DragRectangle(
                color_image, 'RealSense', 640, 480)

        selectinwindow.clearCanvasNDraw(ROI_rec)
        # Show images
        cv2.imshow('RealSense', color_image)
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
