from djitellopy import Tello
import cv2
import time
import pygame

# Initialize pygame for controller support
pygame.init()
pygame.joystick.init()

# Try to initialize the first joystick
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")
else:
    print("No controller found. Proceeding without controller support.")

# Create Tello object and connect
print('Connecting to Tello...')
tello = Tello()
tello.connect()
print('Connected to Tello')

# Start the video stream
print('Starting video stream...')
tello.streamon()

downvision_enabled = True
tello.send_command_with_return('downvision 1')

frame_reader = tello.get_frame_read()

# Create fullscreen window for display
window_name = 'drone'
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
#cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# Battery overlay state
battery_level = None
last_battery_check = 0.0
battery_update_interval = 10.0

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.7
font_thickness = 2
text_color = (255, 255, 255)
text_bg_color = (0, 0, 0)

# Controller deadzone
DEADZONE = 0.1

def update_battery_level():
    global battery_level, last_battery_check
    try:
        battery_level = tello.get_battery()
    except Exception as exc:
        battery_level = None
        print(f'Could not read battery level: {exc}')
    last_battery_check = time.time()

def get_controller_input():
    if joystick is None:
        return 0, 0, 0, 0, False, False, False

    # Update pygame events
    pygame.event.pump()

    # Left stick: lr (X), fb (Y inverted)
    lr = joystick.get_axis(0)
    fb = -joystick.get_axis(1)  # Invert Y axis

    # Right stick: yv (X), ud (Y inverted)
    yv = joystick.get_axis(2)
    ud = -joystick.get_axis(3)  # Invert Y axis

    # Apply deadzone
    lr = lr if abs(lr) > DEADZONE else 0
    fb = fb if abs(fb) > DEADZONE else 0
    ud = ud if abs(ud) > DEADZONE else 0
    yv = yv if abs(yv) > DEADZONE else 0

    # Scale to -100 to 100
    lr = int(lr * 100)
    fb = int(fb * 100)
    ud = int(ud * 100)
    yv = int(yv * 100)

    # Button 0 (A button on Xbox) to toggle camera
    # Button 1 (B button) to takeoff
    # Button 2 (X button) to land
    toggle_camera = joystick.get_button(0)
    takeoff_button = joystick.get_button(1)
    land_button = joystick.get_button(2)

    return lr, fb, ud, yv, toggle_camera, takeoff_button, land_button

update_battery_level()

print('Controls:')
print('Controller: Left stick for left/right/forward/back, Right stick for up/down/yaw')
print('A button (Xbox) or button 0 to toggle camera')
print('B button (Xbox) or button 1 to takeoff')
print('X button (Xbox) or button 2 to land')
print('Keyboard: t to takeoff, l to land, d to toggle camera, ESC to exit')

last_toggle = False

while True:
    img = frame_reader.frame
    if img is None:
        continue

    if time.time() - last_battery_check >= battery_update_interval:
        update_battery_level()

    # Get controller input
    lr, fb, ud, yv, toggle_camera, takeoff_button, land_button = get_controller_input()

    # Send control commands to drone
    tello.send_rc_control(lr, fb, ud, yv)

    # Handle camera toggle
    if toggle_camera and not last_toggle:
        downvision_enabled = not downvision_enabled
        tello.send_command_with_return(f'downvision {1 if downvision_enabled else 0}')
        print(f"Downvision {'enabled' if downvision_enabled else 'disabled'}")
    last_toggle = toggle_camera

    # Handle takeoff
    if takeoff_button:
        tello.takeoff()
        print("Takeoff initiated")

    # Handle land
    if land_button:
        tello.land()
        print("Landing initiated")

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    if battery_level is not None:
        overlay_text = f'Battery: {battery_level}%'
    else:
        overlay_text = 'Battery: unknown'

    overlay_text = f"{overlay_text} | {'Downvision' if downvision_enabled else 'Normal Vision'}"

    # Draw a filled background rectangle for better visibility
    text_size, _ = cv2.getTextSize(overlay_text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, (10, 10), (20 + text_w, 25 + text_h), text_bg_color, cv2.FILLED)
    cv2.putText(img, overlay_text, (15, 30 + text_h // 2), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

    cv2.imshow(window_name, img)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    elif key == ord('d'):
        downvision_enabled = not downvision_enabled
        tello.send_command_with_return(f'downvision {1 if downvision_enabled else 0}')
        print(f"Downvision {'enabled' if downvision_enabled else 'disabled'}")
    elif key == ord('t'):
        tello.takeoff()
        print("Takeoff initiated")
    elif key == ord('l'):
        tello.land()
        print("Landing initiated")

cv2.destroyWindow(window_name)
cv2.destroyAllWindows()
tello.streamoff()

# Quit pygame
pygame.quit()
