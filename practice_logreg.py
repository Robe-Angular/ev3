#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from logreg_ev3 import LogisticPortable
import time, sys, os

# Motors & sensors
lm = LargeMotor(OUTPUT_B); rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'; rm.stop_action = 'brake'
csL = ColorSensor(INPUT_1); csC = ColorSensor(INPUT_2); csR = ColorSensor(INPUT_3)
touch = TouchSensor(INPUT_4)

def wait_press_release():
    while not touch.is_pressed: time.sleep(0.01)
    while touch.is_pressed: time.sleep(0.01)

# Calibration
print("Put sensors on WHITE and press touch...")
wait_press_release()
whiteL, whiteC, whiteR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity
print("Now on BLACK and press touch...")
wait_press_release()
blackL, blackC, blackR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

# Model
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # carpeta de este script
logreg_dir = os.path.join(BASE_DIR, "export_logreg")

print("Cargando Log Reg desde:", logreg_dir)  # opcional, para verificar en consola

logr = LogisticPortable(logreg_dir, mode="calib")
logr.set_calibration(whiteL, whiteC, whiteR, blackL, blackC, blackR)

# Motion map (always include forward to avoid spinning in place)
FWD, TURN = -7, -15
map_cmd = {
    "LEFT":   (FWD - TURN, FWD + TURN),
    "CENTER": (FWD,        FWD),
    "RIGHT":  (FWD + TURN, FWD - TURN),
}

print("Press touch to START...")
wait_press_release()

last_label = "CENTER"
last_log = 0.0
LOG_EVERY = 0.5

try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        # With debounce by confidence (optional)
        label, probs, _ = logr.predict_with_probs(L=L, C=C, R=R)
        probs_sorted = sorted(probs, reverse=True)
        margin = probs_sorted[0] - probs_sorted[1] if len(probs) >= 2 else probs_sorted[0]
        if margin < 0.25:  # low confidence → keep last (or push CENTER)
            cand = last_label if last_label == "CENTER" else "CENTER"
        else:
            cand = label
        last_label = cand

        cmdL, cmdR = map_cmd.get(last_label, (FWD, FWD))
        lm.on(SpeedPercent(cmdL)); rm.on(SpeedPercent(cmdR))

        if touch.is_pressed:  # touch to stop
            break
        time.sleep(0.05)
except KeyboardInterrupt:
    sys.stdout.write("Ctrl+C\n")
finally:
    lm.stop(); rm.stop()
    sys.stdout.write("Motors stopped.\n")
