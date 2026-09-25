#Importing the required libraries:
from djitellopy import Tello
import cv2

# Create Tello Object
tello = Tello()

# Connect to Tello
tello.connect()

# Start the video Stream
tello.streamon()

# Initialize downvision
downvision_enabled = True
tello.send_command_with_return("downvision 1")

# Get the frame reader
frame_reader = tello.get_frame_read()

#Displaying the video stream in a loop:
while True:
    # In reality you want to display frames in a seperate thread. Otherwise
    #  they will freeze while the drone moves.

    # Read a video frame from Tello
    img = frame_reader.frame
    img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    #img= cv2.flip(img,0)

    # Have OpenCV display the Video Frame
    cv2.imshow("drone", img)
#Exiting the loop:
    # If ESC is pressed then stop
    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break
    elif key == ord('d'): # Toggle downvision
        downvision_enabled = not downvision_enabled
        tello.send_command_with_return(f"downvision {1 if downvision_enabled else 0}")
        print(f"Downvision {'enabled' if downvision_enabled else 'disabled'}")

#Cleaning up:
cv2.destroyWindow('drone')
cv2.destroyAllWindows()
tello.streamoff()