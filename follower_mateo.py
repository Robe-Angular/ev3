#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Line follower with automatic calibration and persistent search mode.
Ports:
  Motors:  B (left), C (right)
  Sensors: 1=L, 2=C, 3=R, 4=Touch
Rotates toward the correct side of the detected line (fixed logic).
Compatible with Python 3.5 (EV3).
"""

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time, os, csv

# --- Hardware ---
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
snd = Sound(); leds = Leds()

# --- CSV setup ---
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
csv_path = os.path.join(log_dir, "line_search_" + ts + ".csv")
f = open(csv_path, "w")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","state"])
t0 = time.time()

# --- Calibration ---
def wait_press():
    while not touch.is_pressed:
        time.sleep(0.05)
    while touch.is_pressed:
        time.sleep(0.05)

def avg(sensor, n=10):
    s = 0
    for _ in range(n):
        s += sensor.reflected_light_intensity
        time.sleep(0.01)
    return s / float(n)

def calibrate():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    snd.speak("Place sensors on white and press the button.")
    wait_press()
    white = [avg(csL), avg(csC), avg(csR)]
    print("White:", white)

    snd.speak("Now place sensors on black line and press the button.")
    wait_press()
    black = [avg(csL), avg(csC), avg(csR)]
    print("Black:", black)

    b_avg = sum(black)/3.0
    w_avg = sum(white)/3.0
    delta = w_avg - b_avg
    VER_BLACK = b_avg + 0.20*delta
    VER_GRAY  = b_avg + 0.45*delta
    VER_WHITE = b_avg + 0.70*delta

    snd.speak("Calibration complete.")
    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    print("Thresholds -> BLACK={:.1f}  GRAY={:.1f}  WHITE={:.1f}".format(
        VER_BLACK, VER_GRAY, VER_WHITE))
    return VER_BLACK, VER_GRAY, VER_WHITE

VER_BLACK, VER_GRAY, VER_WHITE = calibrate()

# --- Speeds ---
VEL_HIGH = 80
VEL_MED  = 50
VEL_LOW  = 10

# --- Search mode ---
def search_line(last_side):
    """Rotate indefinitely toward last side until line is found."""
    snd.speak("Searching line")
    leds.set_color('LEFT','ORANGE'); leds.set_color('RIGHT','ORANGE')
    while True:
        if touch.is_pressed:
            snd.speak("Manual stop")
            lm.stop(); rm.stop()
            return False

        L = csL.value(); C = csC.value(); R = csR.value()
        if L < VER_BLACK or C < VER_BLACK or R < VER_BLACK:
            snd.speak("Line reacquired")
            leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
            return True

        if last_side == 'left':
            # giro hacia la izquierda
            lm.run_forever(speed_sp=-VEL_LOW)
            rm.run_forever(speed_sp=VEL_HIGH)
        else:
            # giro hacia la derecha
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=-VEL_LOW)

        time.sleep(0.05)

# --- Main loop ---
snd.speak("Ready. Press button to start.")
wait_press()
snd.speak("Line following started.")
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

running = True
last_side = 'right'

try:
    while running:
        if touch.is_pressed:
            snd.speak("Program stopped.")
            break

        L = csL.value(); C = csC.value(); R = csR.value()
        state = "FORWARD"

        # --- logic (corrected directions) ---
        if L < VER_BLACK and C < VER_BLACK:
            # curva 90° izquierda
            lm.run_forever(speed_sp=-VEL_LOW)
            rm.run_forever(speed_sp=VEL_HIGH)
            last_side = 'left'; state = "TURN_LEFT_90"

        elif R < VER_BLACK and C < VER_BLACK:
            # curva 90° derecha
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=-VEL_LOW)
            last_side = 'right'; state = "TURN_RIGHT_90"

        elif L < VER_BLACK:
            # línea a la izquierda → gira izquierda
            lm.run_forever(speed_sp=VEL_LOW)
            rm.run_forever(speed_sp=VEL_HIGH)
            last_side = 'left'; state = "HARD_LEFT"

        elif R < VER_BLACK:
            # línea a la derecha → gira derecha
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_LOW)
            last_side = 'right'; state = "HARD_RIGHT"

        elif L < VER_GRAY:
            lm.run_forever(speed_sp=VEL_MED)
            rm.run_forever(speed_sp=VEL_HIGH)
            last_side = 'left'; state = "SOFT_LEFT"

        elif R < VER_GRAY:
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_MED)
            last_side = 'right'; state = "SOFT_RIGHT"

        elif L > VER_WHITE and C > VER_WHITE and R > VER_WHITE:
            print("Line lost → searching", last_side)
            lm.stop(); rm.stop()
            if not search_line(last_side):
                running = False
                break
            else:
                continue

        else:
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_HIGH)

        writer.writerow([round(time.time()-t0,3), L, C, R, lm.speed, rm.speed, state])
        f.flush()
        time.sleep(0.1)

except KeyboardInterrupt:
    pass

finally:
    lm.stop(); rm.stop()
    leds.all_off()
    snd.speak("Mission complete")
    f.close()
    print("CSV saved at:", csv_path)
