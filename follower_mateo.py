#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple line follower with automatic calibration.
Compatible with Python 3.5 (EV3).
Uses 3 color sensors (L,C,R) and two motors (B,C).
Records data to CSV in /home/robot/logs/.
"""

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
from ev3dev2.button import Button
import csv, os, time

# --- Hardware ---
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
snd = Sound(); leds = Leds(); btn = Button()

# --- CSV setup ---
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
csv_path = os.path.join(log_dir, "follower_log_" + ts + ".csv")
f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","state"])
t0 = time.time()

# --- Calibration ---
def wait_press():
    while not btn.enter:
        time.sleep(0.05)
    while btn.enter:
        time.sleep(0.05)

def avg(sensor, n=10):
    total = 0
    for _ in range(n):
        total += sensor.reflected_light_intensity
    return total / float(n)

def calibrate():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    snd.speak("Place sensors on white and press the center button.")
    wait_press()
    white = [avg(csL), avg(csC), avg(csR)]
    print("White:", white)

    snd.speak("Now place sensors on black line and press again.")
    wait_press()
    black = [avg(csL), avg(csC), avg(csR)]
    print("Black:", black)

    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    snd.speak("Calibration complete.")
    b_avg = sum(black)/3.0
    w_avg = sum(white)/3.0
    delta = w_avg - b_avg
    VER_BLACK = b_avg + 0.20*delta
    VER_GRAY  = b_avg + 0.45*delta
    VER_WHITE = b_avg + 0.70*delta
    print("Thresholds -> BLACK={:.1f} GRAY={:.1f} WHITE={:.1f}".format(
        VER_BLACK, VER_GRAY, VER_WHITE))
    return VER_BLACK, VER_GRAY, VER_WHITE

VER_BLACK, VER_GRAY, VER_WHITE = calibrate()

# --- Motor speeds ---
VEL_HIGH = 80
VEL_MED  = 50
VEL_LOW  = 10
VEL_REV  = -50

snd.speak("Ready. Press button to start.")
wait_press()
snd.speak("Line following started.")
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

# --- Main loop ---
running = True
try:
    while running:
        L = csL.value()
        C = csC.value()
        R = csR.value()

        state = "FORWARD"

        # 1️⃣ Right 90°
        if L < VER_BLACK and C < VER_BLACK:
            state = "TURN_RIGHT_90"
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_REV)

        # 2️⃣ Left 90°
        elif R < VER_BLACK and C < VER_BLACK:
            state = "TURN_LEFT_90"
            lm.run_forever(speed_sp=VEL_REV)
            rm.run_forever(speed_sp=VEL_HIGH)

        # 3️⃣ Hard correction right
        elif L < VER_BLACK:
            state = "HARD_RIGHT"
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_LOW)

        # 4️⃣ Hard correction left
        elif R < VER_BLACK:
            state = "HARD_LEFT"
            lm.run_forever(speed_sp=VEL_LOW)
            rm.run_forever(speed_sp=VEL_HIGH)

        # 5️⃣ Soft correction right
        elif L < VER_GRAY:
            state = "SOFT_RIGHT"
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_MED)

        # 6️⃣ Soft correction left
        elif R < VER_GRAY:
            state = "SOFT_LEFT"
            lm.run_forever(speed_sp=VEL_MED)
            rm.run_forever(speed_sp=VEL_HIGH)

        # 7️⃣ Lost line
        elif L > VER_WHITE and C > VER_WHITE and R > VER_WHITE:
            state = "LINE_LOST"
            lm.stop(); rm.stop()
            snd.speak("Line lost, stopping.")
            leds.set_color('LEFT','RED'); leds.set_color('RIGHT','RED')
            running = False
            break

        # 8️⃣ Default forward
        else:
            lm.run_forever(speed_sp=VEL_HIGH)
            rm.run_forever(speed_sp=VEL_HIGH)

        # Log to CSV
        writer.writerow([
            round(time.time() - t0, 3),
            L, C, R,
            lm.speed, rm.speed,
            state
        ])
        f.flush()

        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    f.close()
    leds.all_off()
    snd.speak("Mission complete.")
    print("CSV saved at:", csv_path)
