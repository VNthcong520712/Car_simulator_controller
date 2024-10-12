import cv2, math, random
import numpy as np

intersect_turn = None

def calculate_steering_angle(lane_center, img_center, width):
	deviation = lane_center - img_center
	max_dev = img_center
	# Calculate steering angle
	r = 0.55 * width
	angle = math.degrees(math.asin(deviation / r))
	steering_angle = (0.1*angle**2)/25
	steering_angle = steering_angle if angle >= 0 else -steering_angle
	return np.clip(steering_angle, -1, 1)

def detect_intersection(contours, image):
	if cv2.contourArea(contours) > 69000:
		img_left = image[:,:41]
		img_right = image[:,image.shape[1]-41:]
		img_top = image[:80, int(0.3333*image.shape[1]):int(0.6666*image.shape[1])]

		mean_left = np.mean(img_left)
		mean_right = np.mean(img_right)
		mean_top = np.mean(img_top)

		means = [0, 0, 0]
		for id, mean in enumerate((mean_left, mean_top, mean_right), -1):
			if mean > 100:
				means[id + 1] = id*0.83 + 0.0001
		return {"left":means[0], "straight":means[1], "right":means[2]} 
	return {"left":0, "straight":0, "right":0}

def cal_steering(image, turn_range, sign = None, draw = None):
	global intersect_turn
	# Requiment condition
	scan_range = 0.7
	if scan_range >= turn_range:
		raise "scan_range can not larger than turn_range"

	# Inint throttle and steering
	throttle = 0.5
	steering_angle = 0.0

	# Define color range for the gray lane in HSV
	lower_gray = np.array([0,0,50])
	upper_gray = np.array([180,50,150])

	# Kernel (morphological)
	kernel = np.ones([5,5], np.uint8)

	# Height and width
	img_height = image.shape[0]
	img_width = image.shape[1]

	# Center X
	img_center = img_width // 2

	# The scan region
	turn_angle = int(img_height*turn_range)
	scan_region = int(img_height*scan_range) - 80 # 80 is forward checking value

	# Cut image
	scan_img = image[scan_region:, :].copy()

	# Convert to HSV
	hsv_img = cv2.cvtColor(scan_img, cv2.COLOR_BGR2HSV)

	# Create mask for the gray lane
	mask_gray = cv2.inRange(hsv_img, lower_gray, upper_gray)

	# Apply morphological to reduce noise and clean the mask
	mask_gray_clean = cv2.morphologyEx(mask_gray, cv2.MORPH_CLOSE, kernel)
	mask_gray_clean = cv2.morphologyEx(mask_gray_clean, cv2.MORPH_OPEN, kernel)

	# Find contours 
	contours_gray_scan, _ = cv2.findContours(mask_gray_clean[80:, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours_gray_turn, _ = cv2.findContours(mask_gray_clean[int(scan_img.shape[0]*(1- (1-turn_range)/(1-scan_range))):,:], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	if contours_gray_scan and contours_gray_turn:
		# Get the largest contour, migh be lane
		largest_contour_scan = max(contours_gray_scan, key=cv2.contourArea)
		largest_contour_turn = max(contours_gray_turn, key=cv2.contourArea)
		
		# Calculate the centroid
		M = cv2.moments(largest_contour_turn)
		if M['m00'] > 0:
			# X centroid
			lane_center_x = int(M['m10'] / M['m00'])

			# Y centroid
			percentage = (M['m10'] / M['m00'])/480
			lane_center_y = int(turn_angle + percentage*(img_height-turn_angle))

			# Cal steering angle
			steering_angle = calculate_steering_angle(lane_center_x, img_center, img_width)

		intersec = detect_intersection(largest_contour_scan, mask_gray_clean)
		if list(intersec.values()).count(0) < 2:
			if sign:
				steering_angle = intersec[sign]
			else:
				if intersec['straight']:
					steering_angle = intersec["straight"]
				else:
					choice = random.choice(['left', 'right'])
					steering_angle = intersec[choice]
			intersect_turn = False
		else:
			if intersect_turn is False:
				intersect_turn = True
			else:
				intersect_turn = None

	if draw is not None:	
		# Draw the lane center on the frame
		try:
			draw = cv2.circle(draw, (lane_center_x, lane_center_y), 5, (0, 0, 255), -1)
		except:
			pass
		draw = cv2.line(draw, (0, scan_region+80), (img_width, scan_region+80), (255, 0, 0), 2)
		draw = cv2.line(draw, (0, turn_angle), (img_width, turn_angle), (0, 255, 0), 2)
			
	
	return throttle, steering_angle, intersect_turn