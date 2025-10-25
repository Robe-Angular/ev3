# file: claw_keyboard.py
# control the claw using keyboard input over ssh (ascii only)

from ev3dev2.motor import MediumMotor, OUTPUT_A
from time import sleep
import sys, termios, tty

m = MediumMotor(OUTPUT_A)
STEP_DEG = 60
SPEED = 30

print("Keyboard control ready:")
print("a = close, z = open, q = quit")

# setup terminal for raw key reading
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    while True:
        key = sys.stdin.read(1)
        if key == 'a':
            print("close")
            m.on_for_degrees(speed=SPEED, degrees=STEP_DEG)
        elif key == 'z':
            print("open")
            m.on_for_degrees(speed=-SPEED, degrees=STEP_DEG)
        elif key == 'q':
            print("quit")
            m.stop()
            break
        sleep(0.05)
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
