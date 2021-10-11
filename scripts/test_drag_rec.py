import selectinwindow
import sys

import cv2  # Opencv ver 3.1.0 used
import numpy as np

# Set recursion limit
sys.setrecursionlimit(10 ** 9)


# Initialize the  drag object
wName = "select region"
imageWidth = 320
imageHeight = 240
# OR read an image using imread()
image = np.ones([imageHeight, imageWidth, 3], dtype=np.uint8)
image *= 255

# Define the drag object
rectI = selectinwindow.DragRectangle(image, wName, imageWidth, imageHeight)

cv2.namedWindow(rectI.wname)
cv2.setMouseCallback(rectI.wname, selectinwindow.dragrect, rectI)

# keep looping until rectangle finalized
while True:
    # display the image
    cv2.imshow(wName, rectI.image)
    key = cv2.waitKey(1)
    print(key)
    if key & 0xFF == ord('q') or key == 27:
        break

print("Dragged rectangle coordinates")
print(str(rectI.outRect.x) + ',' + str(rectI.outRect.y) + ',' +
      str(rectI.outRect.w) + ',' + str(rectI.outRect.h))

# close all open windows
cv2.destroyAllWindows()
