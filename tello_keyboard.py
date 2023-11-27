# Import necessary libraries
from djitellopy import tello
import KeyPressModule as kp
import time
import cv2

# Initialize the custom keypress module
kp.init()

# Create a Tello drone object and connect to it
me = tello.Tello()
me.connect()

# Print the Tello drone's battery level
print("Battery level:", me.get_battery())

# Start the video stream from the Tello drone
me.streamon()

# Function to get control inputs based on keyboard key presses
def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 20

    # Check for left and right key presses to control left-right movement
    if kp.getKey("LEFT"):
        lr = -speed
    elif kp.getKey("RIGHT"):
        lr = speed

    # Check for up and down key presses to control forward-backward movement
    if kp.getKey("UP"):
        fb = speed
    elif kp.getKey("DOWN"):
        fb = -speed

    # Check for 'w' and 's' key presses to control up-down movement
    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed

    # Check for 'a' and 'd' key presses to control yaw (rotation)
    if kp.getKey("a"):
        yv = -speed
    elif kp.getKey("d"):
        yv = speed

    # Check for 'q' key press to land the drone and wait for 3 seconds
    if kp.getKey("q"):
        me.land()
        time.sleep(3)

    # Check for 'e' key press to take off
    if kp.getKey("e"):
        me.takeoff()

    # Check for 'z' key press to capture an image and save it with a timestamp filename
    if kp.getKey("z"):
        cv2.imwrite(f'{time.time()}.jpg', img)
        time.sleep(0.3)

    # Return the control inputs as a list [lr, fb, ud, yv]
    return [lr, fb, ud, yv]

# Main execution loop
while True:
    # Get control inputs based on keyboard key presses
    vals = getKeyboardInput()

    # Send the control inputs to the Tello drone
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    # Capture a frame from the drone's video stream
    img = me.get_frame_read().frame

    # Resize the captured image to a smaller size (360x240 pixels)
    img = cv2.resize(img, (360, 240))

    # Display the resized image in a window named "Image"
    cv2.imshow("Image", img)

    # Wait for a key press for a short duration (1 millisecond)
    cv2.waitKey(1)