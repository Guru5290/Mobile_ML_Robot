# # Python code for Multiple Color Detection obtained from geeks for geeks website.

# #modified 

# import os
# import numpy as np 
# import cv2 


# # Capturing video through webcam 
# rtsp_url = f"rtsp://10.21.16.67:8554/cam"
# webcam = cv2.VideoCapture(rtsp_url) 


# # Start a while loop 
# while(1): 
	
# 	# Reading the video from the 
# 	# webcam in image frames 
# 	_, imageFrame = webcam.read() 

# 	# Convert the imageFrame in 
# 	# BGR(RGB color space) to 
# 	# HSV(hue-saturation-value) 
# 	# color space 
# 	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

# 	# Set range for red color and 
# 	# define mask 
# 	red_lower = np.array([136, 87, 111], np.uint8) 
# 	red_upper = np.array([180, 255, 255], np.uint8) 
# 	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
	
#     # Set range for yellow color and 
# 	# define mask 
# 	yellow_lower = np.array([20, 100, 100], np.uint8) 
# 	yellow_upper = np.array([30, 255, 255], np.uint8) 
# 	yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper) 

# 	# Set range for green color and 
# 	# define mask 
# 	green_lower = np.array([72, 52, 25], np.uint8) 
# 	green_upper = np.array([102, 255, 255], np.uint8) 
# 	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

# 	# Set range for blue color and 
# 	# define mask 
# 	blue_lower = np.array([94, 80, 2], np.uint8) 
# 	blue_upper = np.array([120, 255, 255], np.uint8) 
# 	blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
	
# 	# Morphological Transform, Dilation 
# 	# for each color and bitwise_and operator 
# 	# between imageFrame and mask determines 
# 	# to detect only that particular color 
# 	kernel = np.ones((5, 5), "uint8") 
	
# 	# For red color 
# 	red_mask = cv2.dilate(red_mask, kernel) 
# 	res_red = cv2.bitwise_and(imageFrame, imageFrame, 
# 							mask = red_mask) 
	
#     # For yellow color 
# 	yellow_mask_mask = cv2.dilate(yellow_mask, kernel) 
# 	res_yellow = cv2.bitwise_and(imageFrame, imageFrame, 
# 							mask = yellow_mask) 
	
# 	# For green color 
# 	green_mask = cv2.dilate(green_mask, kernel) 
# 	res_green = cv2.bitwise_and(imageFrame, imageFrame, 
# 								mask = green_mask) 
	
# 	# For blue color 
# 	blue_mask = cv2.dilate(blue_mask, kernel) 
# 	res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
# 							mask = blue_mask) 

# 	# Creating contour to track red color 
# 	contours, hierarchy = cv2.findContours(red_mask, 
# 										cv2.RETR_TREE, 
# 										cv2.CHAIN_APPROX_SIMPLE) 
	
# 	for pic, contour in enumerate(contours): 
# 		area = cv2.contourArea(contour) 
# 		if(area > 30000): 
# 			x, y, w, h = cv2.boundingRect(contour) 
# 			imageFrame = cv2.rectangle(imageFrame, (x, y), 
# 									(x + w, y + h), 
# 									(0, 0, 255), 2) 
			
# 			cv2.putText(imageFrame, "Red Colour", (x, y), 
# 						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
# 						(0, 0, 255))	 

# 	# Creating contour to track green color 
# 	contours, hierarchy = cv2.findContours(green_mask, 
# 										cv2.RETR_TREE, 
# 										cv2.CHAIN_APPROX_SIMPLE) 
	
# 	for pic, contour in enumerate(contours): 
# 		area = cv2.contourArea(contour) 
# 		if(area > 30000): 
# 			x, y, w, h = cv2.boundingRect(contour) 
# 			imageFrame = cv2.rectangle(imageFrame, (x, y), 
# 									(x + w, y + h), 
# 									(0, 255, 0), 2) 
			
# 			cv2.putText(imageFrame, "Green Colour", (x, y), 
# 						cv2.FONT_HERSHEY_SIMPLEX, 
# 						1.0, (0, 255, 0)) 
	
#     # Creating contour to track yellow color 
# 	contours, hierarchy = cv2.findContours(yellow_mask, 
# 										cv2.RETR_TREE, 
# 										cv2.CHAIN_APPROX_SIMPLE) 
	
# 	for pic, contour in enumerate(contours): 
# 		area = cv2.contourArea(contour) 
# 		if(area > 30000): 
# 			x, y, w, h = cv2.boundingRect(contour) 
# 			imageFrame = cv2.rectangle(imageFrame, (x, y), 
# 									(x + w, y + h), 
# 									(0, 255, 255), 2) 
			
# 			cv2.putText(imageFrame, "Yellow Color", (x, y), 
# 						cv2.FONT_HERSHEY_SIMPLEX, 
# 						1.0, (0, 255, 255)) 

# 	# Creating contour to track blue color 
# 	contours, hierarchy = cv2.findContours(blue_mask, 
# 										cv2.RETR_TREE, 
# 										cv2.CHAIN_APPROX_SIMPLE) 
# 	for pic, contour in enumerate(contours): 
# 		area = cv2.contourArea(contour) 
# 		if(area > 30000): 
# 			x, y, w, h = cv2.boundingRect(contour) 
# 			imageFrame = cv2.rectangle(imageFrame, (x, y), 
# 									(x + w, y + h), 
# 									(255, 0, 0), 2) 
			
# 			cv2.putText(imageFrame, "Blue Colour", (x, y), 
# 						cv2.FONT_HERSHEY_SIMPLEX, 
# 						1.0, (255, 0, 0)) 
			
# 	# Program Termination 
# 	cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
# 	if cv2.waitKey(10) & 0xFF == ord('q'): 
# 		webcam.release() 
# 		cv2.destroyAllWindows() 
# 		break


import os
import numpy as np 
import cv2 

# Capturing video through RTSP stream 
rtsp_url = f"rtsp://10.122.180.67:8554/cam"
webcam = cv2.VideoCapture(rtsp_url) 

# Start a while loop 
while True: 
	
    # Read video frame 
    ret, imageFrame = webcam.read() 
    if not ret:
        print("Failed to grab frame")
        break

    # Convert BGR to HSV 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    # ---- RED ---- (two ranges, then combine)
    red_lower1 = np.array([0, 120, 70]) 
    red_upper1 = np.array([10, 255, 255]) 
    red_lower2 = np.array([170, 120, 70]) 
    red_upper2 = np.array([180, 255, 255]) 

    red_mask1 = cv2.inRange(hsvFrame, red_lower1, red_upper1) 
    red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2) 
    red_mask = red_mask1 + red_mask2

    # ---- YELLOW ----
    yellow_lower = np.array([20, 100, 100]) 
    yellow_upper = np.array([30, 255, 255]) 
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper) 

    # ---- GREEN ---- (improved range)
    green_lower = np.array([35, 52, 72]) 
    green_upper = np.array([85, 255, 255]) 
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

    # ---- BLUE ----
    blue_lower = np.array([90, 80, 40]) 
    blue_upper = np.array([130, 255, 255]) 
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

    # Kernel for dilation 
    kernel = np.ones((5, 5), "uint8") 

    # Apply dilations 
    red_mask = cv2.dilate(red_mask, kernel) 
    yellow_mask = cv2.dilate(yellow_mask, kernel) 
    green_mask = cv2.dilate(green_mask, kernel) 
    blue_mask = cv2.dilate(blue_mask, kernel) 

    # ---- Draw contours for each color ----
    area_contours = {"Red":0,"Green":0, "Yellow":0, "Blue":0,  }

    def detect_and_draw(mask, color_bgr, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        for contour in contours: 
            area = cv2.contourArea(contour) 
            if area > 30000: 
                area_contours[label]=area

                x, y, w, h = cv2.boundingRect(contour) 
                cv2.rectangle(imageFrame, (x, y), (x + w, y + h), color_bgr, 2) 
                cv2.putText(imageFrame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, color_bgr, 2)

    # detect_and_draw(red_mask,   (0, 0, 255), "Red")
    # detect_and_draw(green_mask, (0, 255, 0), "Green")
    detect_and_draw(yellow_mask,(0, 255, 255), "Yellow")
    detect_and_draw(blue_mask,  (255, 0, 0), "Blue")
    # print(area_contours)

    print(max(area_contours, key=area_contours.get), area_contours)

    # Show result 
    cv2.imshow("Multiple Color Detection in Real-Time", imageFrame) 

    # Quit on 'q' 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        break

# Cleanup 
webcam.release() 
cv2.destroyAllWindows()
