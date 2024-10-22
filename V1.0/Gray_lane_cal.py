import cv2, math, random
import numpy as np

# The checking variable, check if the turning done or didn't do
intersect_turn = None
throttle = 0.5

def calculate_steering_angle(lane_center, img_center, width):
	deviation = lane_center - img_center
	# Calculate the steering angle (Paste your fomular here)
	r = 0.55 * width
	angle = math.degrees(math.asin(deviation / r))
	steering_angle = 1.5*(angle/25)**2
	steering_angle = steering_angle if angle >= 0 else -steering_angle
	return steering_angle

def detect_intersection(contours, image):
	# With 0.3 the last of img_height, we get the differece between normal lane and intersection if larger than 69000
	# If you change the scaning range, you need to adjust the number 69000 with your experimentation
	out = []
	if cv2.contourArea(contours) > 69000:
		# Three bliock is checked the posibility direction
		img_left = image[:,:41]
		img_right = image[:,image.shape[1]-41:]
		img_top = image[:80,: ]

		black_cons, _ = cv2.findContours(cv2.bitwise_not(img_top), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		black_cons = [x for x in black_cons if cv2.contourArea(x) > 2000]

		# Cal the mean to know it is real direct or non
		means= {'left':np.mean(img_left), 'straight':len(black_cons), 'right':np.mean(img_right)}
		threshold = {'left':100, 'straight':2, 'right':100} # The threshold to check each direction
		for dir in means.keys():
			if threshold[dir] <= means[dir]:
				out.append(dir)		
	return out

def blinker(dir, draw = None): # Display the movement directed 
	if draw is not None:
		if dir == 'left':
			cv2.arrowedLine(draw, (int(0.3*draw.shape[1]), int(0.8*draw.shape[0])), (int(0.25*draw.shape[1]), int(0.8*draw.shape[0])), (92,42,130), 5)
		elif dir == 'right':
			cv2.arrowedLine(draw, (int(0.7*draw.shape[1]), int(0.8*draw.shape[0])), (int(0.75*draw.shape[1]), int(0.8*draw.shape[0])), (92,42,130), 5)
		elif dir == 'straight':
			cv2.arrowedLine(draw, (draw.shape[1]//2, int(0.8*draw.shape[0])), (draw.shape[1]//2, int(0.75*draw.shape[0])), (92,42,130), 5)

class Movement: # Movement class
	def __init__(self):
		self.sign = []
		self.intersec_direct = []
		self.steering_angle = 0.0
		self.left_right_most_x = []
		self.processing = None

	def update(self, sign = None, intersec_direct = None, steering_angle = None, left_right_most_x = None): # Update new data 
		'''
		Object is a list object that need to be updated
		Data is a correspond data for those object
		'''
		if sign is not None:
			self.sign = sign
			# self.processing = None
		if intersec_direct is not None:
			self.intersec_direct = intersec_direct
		if steering_angle is not None:
			self.steering_angle = steering_angle
		if left_right_most_x is not None:
			self.left_right_most_x = left_right_most_x
	
	# Execute the signal 
	def straight(self, img_cen, img_wid, draw):
		if 'left' in self.intersec_direct and 'right' in self.intersec_direct:
			self.steering_angle = 0
		else:
			a = lambda x: calculate_steering_angle(x, img_cen, img_wid)
			if 'left' in self.intersec_direct:
				self.steering_angle = a(self.left_right_most_x[1]-220)
			elif 'right' in self.intersec_direct:
				self.steering_angle = a(self.left_right_most_x[0]+220)
		self.processing = 'straight'
		blinker(self.processing, draw)
		

	def left(self, img_cen, img_wid, draw):
		self.steering_angle += -0.73
		self.processing = 'left'
		blinker(self.processing, draw)

	def right(self, img_cen, img_wid, draw):
		self.steering_angle += 0.73
		self.processing = 'right'
		blinker(self.processing, draw)
	
	# Main proceduce
	def process(self, im_cen, im_wi, draw):
		execute = {'left': self.left, 'straight':self.straight, 'right':self.right}

		if len(self.intersec_direct) <= 1: # The straight but in the turn so that the area reach the threshold
			if isinstance(self.processing, str):
				self.processing = True
			elif self.processing is True:
				self.processing = None

		elif len(self.intersec_direct) == 2: # The 3-intersection, there are two choice
			if isinstance(self.processing, str):
				execute[self.processing](im_cen, im_wi, draw)
			else:
				if self.sign: # if this intersect has sign
					if self.sign in self.intersec_direct: # if this sign is correct
						execute[self.sign](im_cen, im_wi, draw)
					elif 'no_' in self.sign: # If it is a forbidden sign
						if 'straight' in self.intersec_direct: # If go straight is present, sign is invalid
							execute['straight'](im_cen, im_wi, draw)
						else: # Force the sign, one only is avaiable
							execute['left'](im_cen, im_wi, draw) if self.sign == 'no_right' else execute['right'](im_cen, im_wi, draw)
					else: # The fake sign
						if 'straight' in self.intersec_direct: # If go straight is present, sign is invalid
							execute['straight'](im_cen, im_wi, draw)
						else: # Chose the random sign
							choice = random.choice(self.intersec_direct)
							execute[choice](im_cen, im_wi, draw)
				else:
					if 'straight' in self.intersec_direct: # If go straight is present, sign is invalid
						execute['straight'](im_cen, im_wi, draw)
					else: # Chose the random sign
						choice = random.choice(self.intersec_direct)
						execute[choice](im_cen, im_wi, draw)
		elif len(self.intersec_direct) == 3: # The 4-intersection
			if isinstance(self.processing, str):
				execute[self.processing](im_cen, im_wi, draw)
			else:
				if self.sign and self.sign in self.intersec_direct: # If the sign match the direction
					execute[self.sign](im_cen, im_wi, draw)
				else:
					execute['straight'](im_cen, im_wi, draw)	
					
		return self.steering_angle		
					
wheel = Movement() # call the class

def cal_steering(image, turn_range, sign = None, draw = None):
	global throttle,intersect_turn
	# Requiment condition
	scan_range = 0.7 # Scannig range, if you change this number, you must change the area in detect_intersection
	if scan_range >= turn_range:
		raise "scan_range can not larger than turn_range"

	# Inint throttle and steering
	if sign == 'stop':
		throttle = 0
		sign = None
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
	scan_region = int(img_height*scan_range) - 80 # 80 is the top block checking height

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
		S = cv2.moments(largest_contour_scan)
		if M['m00'] > 0 and S['m00'] > 0:
			# X centroid
			lane_center_x = int(M['m10'] / M['m00'])

			# Find the leftmost point
			leftmost = tuple(largest_contour_turn[largest_contour_turn[:, :, 0].argmin()][0])

			# Find the rightmost point
			rightmost = tuple(largest_contour_turn[largest_contour_turn[:, :, 0].argmax()][0])

			# Cal steering angle
			steering_angle = calculate_steering_angle(lane_center_x, img_center, img_width)

		# Intersection checking
		intersec = detect_intersection(largest_contour_scan, mask_gray_clean)
		wheel.update(sign, intersec, steering_angle, [leftmost[0], rightmost[0]])
		steering_angle = wheel.process(img_center, img_width, draw)
		intersect_turn = wheel.processing if not isinstance(wheel.processing, str) else False
		
	if draw is not None:	
		# Draw the lane center on the frame
		try:
			cv2.circle(draw, (lane_center_x, int(0.95*img_height)), 5, (0, 0, 255), -1)
		except:
			pass
		cv2.line(draw, (0, scan_region+80), (img_width, scan_region+80), (255, 0, 0), 2) # The scan region line
		cv2.line(draw, (0, turn_angle), (img_width, turn_angle), (0, 255, 0), 2) # The turning decision line		
	
	return throttle, steering_angle, intersect_turn