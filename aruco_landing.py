from djitellopy import Tello
import cv2
import numpy as np
import time
import datetime
import os
import argparse
import imutils
import math                           
from imutils.video import VideoStream    
from imutils.video import FPS      
import sys      

me = Tello()
me.connect()
print(me.get_battery())
me.streamon()
me.takeoff()
me.send_command_with_return("downvision 1")
me.send_rc_control(0, 0, 25, 0)
time.sleep(1)

w, h = 640, 480

pid = [0.2, 0.2, 0]
pError = 0
pError2=0

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def findAruco(img):
    img = me.get_frame_read().frame
    
    img = cv2.resize(img,(648,488))

    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    
    aruco_centers = []
    aruco_areas = []
    
    for marker_corners in corners:
        cx = int(np.mean(marker_corners[0][:, 0]))
        cy = int(np.mean(marker_corners[0][:, 1]))
        area = cv2.contourArea(marker_corners[0])
        
        aruco_centers.append([cx, cy])
        aruco_areas.append(area)
        
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
    
    if len(aruco_areas) != 0:
        i = aruco_areas.index(max(aruco_areas))
        #print(aruco_centers)
        return img, [aruco_centers[i], aruco_areas[i]]
    else:
        return img, [[0, 0], 0]
    
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()








def trackAruco(info, w, pid, pError,pError2):
    area = info[1]
    x, y = info[0]
    updown = 0
    forwardback=0
    error = x - w // 2 #h w yerini degistirdim
    error2= y - h // 2
    leftright = pid[0] * error2 + pid[1] * (error2 - pError2)
    leftright =  int(np.clip(leftright, -30, 30))

    forwardback = pid[0] * error + pid[1] * (error - pError)
    forwardback =  int(np.clip(forwardback, -30, 30))

    
    if area > 50000 and area<70000:
        updown = 0

    elif area < 50000 and area>500:
        updown = -10
    
    if x == 0:
        leftright = 0
        error = 0
    if y== 0:
        forwardback=0
        error2=0
    
    me.send_rc_control(-leftright,-forwardback , updown, 0)
    print(-leftright,"---",-forwardback,"--", updown )
    #print("x=", x,"----","y=", y)
    #time.sleep(1)
    
    return error, error2

while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))
    img, info = findAruco(img)
    pError,pError2 = trackAruco(info, w, pid, pError,pError2)
    

    	# detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img,
		arucoDict, parameters=arucoParams)

    	# verify *at least* one ArUco marker was detected
    if len(corners) > 0:
		# flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
                        # draw the bounding box of the ArUCo detection
            cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the frame
            cv2.putText(img, str(markerID),
                (topLeft[0], topLeft[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            # Calculate the area of the rectangle
            topRight = np.array(topRight)
            topLeft = np.array(topLeft)
            bottomRight = np.array(bottomRight)
            bottomLeft = np.array(bottomLeft)
            widthAruco = np.linalg.norm(topRight - topLeft)
            heightAruco = np.linalg.norm(bottomLeft - topLeft)
            area = widthAruco * heightAruco

            # Draw the calculated area on the frame
            cv2.putText(img, f"Area: {area:.2f}", (topLeft[0], topLeft[1] + 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    area=info[1]
    if area > 20000 and area<25000:
        me.land
        break


    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    #print("Center", info[0], "Area", info[1])
    cv2.imshow("Output", img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        me.end()
        break
