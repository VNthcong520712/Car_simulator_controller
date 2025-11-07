import numpy as np 

RADIUS_RATIO = 0.55
CONTOUR_AREA = 69000

SCAN_RANGE = 0.7
KERNEL = np.ones([5,5], np.uint8)

LOWER_BLACK = np.array([0,0,0])
UPPER_BLACK = np.array([121,126,129])