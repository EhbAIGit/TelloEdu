from djitellopy import Tello
import cv2
import numpy as np
import time
import datetime
import os
import imutils  
from simple_pid import PID
import PoseModule as pm

tello = Tello()
detector = pm.poseDetector()



def main():
    

    tello.connect()
    tello.streamon()

    global send_rc_control
    global for_back_velocity
    global left_right_velocity
    global up_down_velocity
    global yaw_velocity
    global send_rc_control
    global space

    for_back_velocity = 0
    left_right_velocity = 0
    up_down_velocity = 0
    yaw_velocity = 0
    space = 100

    inis = False
    starttime = 0
    say = 0

    send_rc_control = False
    OVERRIDE = True

    humandetect = False

    status = "KEYBOARD CONTROL"
    controlmode = "MANUAL"

    pid_yaw = PID(0.2, 0.00005, 0.01,setpoint=0,output_limits=(-100,100))
    pid_throttle = PID(0.2, 0.00001, 0.01,setpoint=0,output_limits=(-80,100))
    pid_pitch = PID(0.2, 0.00005, 0.01,setpoint=0,output_limits=(-60,60))


    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame', 960, 720)

    frame_read = tello.get_frame_read()
    cap = cv2.VideoCapture(0)
        
    while True:
        update()
 
        frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
        frameRet = frame_read.frame

        img = detector.findPose(frameRet)
        lmList = detector.findPosition(img, draw=False)

        if len(lmList) !=0:

            humandetect = True


            if lmList[14][2] < lmList[12][2] and lmList[20][1] > lmList[14][1] and lmList[13][2] > lmList[11][2]: #right elbow flexed up
                #controlcommand = "right"
                OVERRIDE = False


            elif lmList[14][2] < lmList[12][2] and lmList[20][1] < lmList[14][1] and lmList[13][2] > lmList[11][2]: #right arm extended up
                controlcommand = "left"

            elif lmList[20][2] < lmList[0][2] and lmList[19][2] < lmList[0][2]:
                #if (lmList[19][1]-lmList[20][1]) < (lmList[11][1] - lmList[12][1]):
                    #controlcommand = "forward"
                if (lmList[19][1]-lmList[20][1]) > (lmList[11][1] - lmList[12][1]):
                    controlcommand = "backward"
                else:
                    controlcommand = ""

            elif lmList[20][1] > lmList[19][1] and lmList[20][2] > lmList[0][2] and lmList[19][2] > lmList[0][2]: #hands crossed
                controlcommand = "land"

            elif lmList[13][2] < lmList[11][2] and lmList[19][1] < lmList[13][1]:  #left arm flexed
                #OVERRIDE = True
                controlcommand = "forward"

            elif lmList[13][2] < lmList[11][2] and lmList[19][1] > lmList[13][1]:# left arm extended up
                #OVERRIDE = False        
                controlcommand = "right"
            else:
                controlcommand = ""

        else:
            humandetect = False



        endtime = time.time()
        zaman = endtime - starttime
        if inis == True and zaman > 4 and say<2:
            tello.land()
            say = say + 1
            status = "LANDING"

        elif inis == True and zaman < 4 and say<2:
            left_right_velocity = 0
            yaw_velocity = 0
            up_down_velocity = 0
            for_back_velocity = 0
            status = "LANDING"



        k = cv2.waitKey(20)

        if k == ord('t'):
            print("Taking Off")
            #tello.takeoff()
            tello.get_battery()
            send_rc_control = True

            # Press L to land
        if k == ord('l'):
            print("Landing")
            tello.land()
            time.sleep(1.7)
            send_rc_control = False

            # Press Backspace for controls override
        if k == 8:
            if not OVERRIDE:
                OVERRIDE = True
                print("OVERRIDE ENABLED")
            else:
                OVERRIDE = False
                print("OVERRIDE DISABLED")

        if OVERRIDE:
            status = "KEYBOARD CONTROL"

                # S & W to fly forward & back
            if k == ord('w'):
                for_back_velocity = int(20)
                controlmode = "MANUAL-FORWARD"
            elif k == ord('s'):
                for_back_velocity = -int(20)
                controlmode = "MANUAL-BACKWARD"
            else:
                for_back_velocity = 0

                # a & d to pan left & right
            if k == ord('d'):
                yaw_velocity = int(20)
                controlmode = "MANUAL-RIGHT YAW"
            elif k == ord('a'):
                yaw_velocity = -int(20)
                controlmode = "MANUAL-LEFT YAW"
            else:
                yaw_velocity = 0

                # Q & E to fly up & down
            if k == ord('e'):
                up_down_velocity = int(20)
                controlmode = "MANUAL-UP"
            elif k == ord('q'):
                up_down_velocity = -int(20)
                controlmode = "MANUAL-DOWN"
            else:
                up_down_velocity = 0

                # c & z to fly left & right
            if k == ord('c'):
                left_right_velocity = int(20)
                controlmode = "MANUAL-RIGHT ROLL"
            elif k == ord('z'):
                left_right_velocity = -int(20)
                controlmode = "MANUAL-LEFT ROLL"
            else:
                left_right_velocity = 0

            # Quit the software
        if k == 27:
            should_stop = True
            break


        if send_rc_control and not OVERRIDE and humandetect:

            status = "GUIDED"

            targ_cord_x = lmList[0][1]
            targ_cord_y = lmList[0][2]
            targ_cord_w = lmList[11][1] - lmList[12][1]

            xoff = int(targ_cord_x - 480)
            yoff = int(360-targ_cord_y)
            zoff = int(space-targ_cord_w)

            yaw_velocity = int(-pid_yaw(xoff))
            up_down_velocity = int(-pid_throttle(yoff))
            for_back_velocity = int(-pid_pitch(zoff))


            if controlcommand == "right":
                left_right_velocity = int(20)
                controlmode = "RIGHT"                   

            elif controlcommand == "left":
                left_right_velocity = -int(20)
                controlmode = "LEFT"

            elif controlcommand == "forward":
                for_back_velocity = int(20)
                space = targ_cord_w
                controlmode = "FORWARD"

            elif controlcommand == "backward":
                for_back_velocity = -int(20)
                space = targ_cord_w
                controlmode = "BACKWARD"

            elif controlcommand == "land":
                inis = True
                say = 1
                starttime = time.time()
                status = "LANDING"
                controlmode = None
                send_rc_control = False


            else:
                left_right_velocity = 0
                controlmode = "NONE"


        if send_rc_control == True:
            cv2.putText(img, status, (0, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(img, controlmode, (0, 100), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2, cv2.LINE_AA)
        elif send_rc_control == False and inis == True:
            cv2.putText(img, "LANDING", (0, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2, cv2.LINE_AA)

        img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        cv2.imshow('frame',img)
        key = cv2.waitKey(1) & 0xFF

    cv2.destroyAllWindows()

    tello.end()


def update():
    if send_rc_control == True:
        tello.send_rc_control(left_right_velocity, for_back_velocity, up_down_velocity,
                                       yaw_velocity)


if __name__ == '__main__':
    main()

