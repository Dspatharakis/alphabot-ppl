from imutils import paths
import numpy as np
import imutils
import cv2
import math
from not_found_error import *

# initialize the known object width (cm), which in this case is the beacon
KNOWN_WIDTH = 8.5

# initialize camera-lenses real world distance (cm) from robot centre
DIST_FROM_CENTRE = 7

PIC_CENTRE_WIDTH = 1296 # x-centre-coordinate of the 2596x1944 resolution picture
FOCAL_LENGTH = 4305

# define the list of HSV boundaries (red, blue, purple, yellow, orange)
# IMPORTANT: colour boundaries should be in HSV: H (0-180) | S (0-255) | V (0-255)
COLOR_BOUNDARIES = [
    [([110, 120, 115], [180, 200, 255])], 
    [([80, 100, 100], [110, 255, 255])], 
    [([125, 10, 50], [160, 65 , 155])], 
    [([20, 75, 125], [35, 255, 255])], 
    [([5, 85, 100], [20, 230, 255])], 
]

class Dna(object):
    def detect_color(self, image):
        # transfer to HSV colour space
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	color = 0
	curr_width = 0	
	# loop over the boundaries, find the most fitting color 
	for boundaries in COLOR_BOUNDARIES:
		print("")
    		if (color == 0):
        		print("Looking for the Red Beacon...")
    		elif (color == 1):
        		print("Looking for the Blue Beacon...")
    		elif (color == 2):
        		print("Looking for the Purple Beacon...")
    		elif (color == 3):
        		print("Looking for the Yellow Beacon...")
    		else:
        		print("Looking for the Orange Beacon...")		
    		for (lower, upper) in boundaries:
			# create NumPy arrays from the boundaries
        		lower = np.array(lower, dtype = "uint8")
        		upper = np.array(upper, dtype = "uint8") 
        		# find the colors within the specified boundaries and apply
        		# the mask
        		mask = cv2.inRange(image, lower, upper)
                        # If searching for red, combine masks
                        if (color == 0):
                            mask = mask | cv2.inRange(image, np.array([0, 140, 115]), np.array([20, 200, 200]))
 
                        try:
				# find contour based on colour
        	                cnts = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        		    	cnts = imutils.grab_contours(cnts)
        		    	c = max(cnts, key = cv2.contourArea)
			    	marker = cv2.minAreaRect(c)

			    	# Always looking for cylinders sitting on their bottom
            		    	if (marker[1][0] < marker[1][1]): 
                	    		width = marker[1][0]
                			height = marker[1][1]
                			angle = marker[2]
            			else:
                			# Cannot be sitting on their side
                			raise ValueError
            			print("Candidate contour width, height and angle:")
            			print width,height,angle
            
            			# Check if feasible beacon marker
            			# 1: width to height ratio must be within the range of [0.37, 0.45]
            			if not((width/height >= 0.37) and (width/height <= 0.45)):
                			raise ValueError
            			# 2: angle must be within the range of [-5, 5]
            			elif not(angle >= -5 and angle <= 5):
                			raise ValueError 
			
                        except ValueError:
				print('Not found!')
                                width = 0
            			continue	
		if (width > curr_width):
            		curr_color = color
            		curr_width = width
			curr_cnt = c
                        break

    		color += 1

	if (curr_width == 0):
    	    print "\nNo Beacon detected!"
            raise NotFoundError
    	    exit()


	if (curr_color == 0):
    		print("\nRed Beacon identified!")
	elif (curr_color == 1):
    		print("\nBlue Beacon identified!")
	elif (curr_color == 2):
    		print("\nPurple Beacon identified!")
	elif (curr_color == 3):
    		print("\nYellow Beacon identified!")
	else:
    		print("\nOrange Beacon identified!")	
        
        return curr_cnt, curr_color

    def find_marker(self, image):
        # detect the beacon color and the respective contour based on it
        cnt, color = self.detect_color(image)   
        
        # compute the bounding box of the of the color region and return it 
        return (cv2.minAreaRect(cnt), cnt, color)

    def find_angle(self, cnt, distance):
        # Find left extreme point
        leftmost = tuple(cnt[cnt[:,:,0].argmin()][0])
        # Find right extreme point
        rightmost = tuple(cnt[cnt[:,:,0].argmax()][0])
        # Calculate centre of contour
        beac_centre = (rightmost[0] + leftmost[0])/2
        beac_pixel_width = rightmost[0] - leftmost[0]
        beac_real_width = KNOWN_WIDTH # cm

        cm_per_pixel = float(beac_real_width) / beac_pixel_width
        beac_pixel_dst_centre = beac_centre - PIC_CENTRE_WIDTH
        beac_real_dst_centre = cm_per_pixel * beac_pixel_dst_centre

        a = float(beac_real_dst_centre)
        b = float(distance)
        c = a / b
        angle = math.asin(c)
        return angle

    def find_distance(self, knownWidth, focalLength, perWidth, distFromCentre):
        # compute and return the distance from the maker to the camera
        return ((knownWidth * focalLength) / perWidth) + distFromCentre

    def find_distance_and_angle(self, imagePath):
        print("Checking image: " + imagePath)
        # load the image, find the marker in the image, then compute the
        # distance to the marker from the camera
        image = cv2.imread(imagePath)
        marker, contour, color = self.find_marker(image)
        cms = self.find_distance(KNOWN_WIDTH, FOCAL_LENGTH, marker[1][0], DIST_FROM_CENTRE)
        angle = self.find_angle(contour, cms)
        print("Calculated Distance: %.2f cm" % cms)
        print("Calculated Angle: %.2f Deg" % math.degrees(angle))
        print("Color Code: " + str(color))

        return cms, math.degrees(angle), color 
