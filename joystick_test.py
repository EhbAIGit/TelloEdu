import pygame
import time

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
    print("No controller found. Exiting.")
    exit()

# Controller deadzone
DEADZONE = 0.1

print("Testing joystick inputs. Press Ctrl+C to exit.")
print("Axes: lr, fb, ud, yv")
print("Buttons: toggle_camera, takeoff, land")

try:
    while True:
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

        # Buttons
        toggle_camera = joystick.get_button(0)
        takeoff_button = joystick.get_button(1)
        land_button = joystick.get_button(2)

        # Print commands
        print(f"RC Control: lr={lr}, fb={fb}, ud={ud}, yv={yv} | Buttons: A={toggle_camera}, B={takeoff_button}, X={land_button}")

        time.sleep(0.1)  # Small delay to avoid flooding

except KeyboardInterrupt:
    print("Exiting...")

# Quit pygame
pygame.quit()