# import necessary packages
import imutils
import cv2
import numpy as np

def objCenter(rects, frameCenter):
	# check to see if a object was found
	if len(rects) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(rects, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		objectX = center[0]
		objectY = center[1]

		# return the center (x, y)-coordinates of the face
		return ((objectX, objectY), radius)

	# otherwise no faces were found, so return the center of the
	# frame
	return (frameCenter, None)
