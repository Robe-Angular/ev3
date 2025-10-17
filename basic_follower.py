#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
import time, os, csv

lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'
rm.stop_action = 'brake'

csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)

# ---------- Calibración rápida ----------
def wait_press_release():
    while not touch.is_pressed: time.sleep(0.01)
    while touch.is_pressed: time.sleep(0.01)

print("Coloca sobre BLANCO y pulsa...")
wait_press_release()
whiteL, whiteC, whiteR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

print("Ahora sobre NEGRO (línea) y pulsa...")
wait_press_release()
blackL, blackC, blackR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

thL = (whiteL + blackL)/2
thC = (whiteC + blackC)/2
thR = (whiteR + blackR)/2
print("Umbrales:", thL, thC, thR)

print("Pulsa otra vez para empezar.")
wait_press_release()

# ---------- CSV ----------
os.makedirs("/home/robot/logs", exist_ok=True)
path = f"/home/robot/logs/line_simple_{time.strftime('%Y%m%d_%H%M%S')}.csv"
f = open(path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","label"])

# ---------- Loop ----------
BASE = 15
TURN = 10
t0 = time.time()

try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        # lógica simple de seguimiento
        if C < thC:              # centro sobre la línea
            cmdL, cmdR, label = BASE, BASE, "STRAIGHT"
        elif L < thL:            # línea a la izquierda
            cmdL, cmdR, label = BASE - TURN, BASE + TURN, "LEFT"
        elif R < thR:            # línea a la derecha
            cmdL, cmdR, label = BASE + TURN, BASE - TURN, "RIGHT"
        else:                    # línea perdida
            cmdL, cmdR, label = -TURN, TURN, "SEARCH"

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        writer.writerow([round(time.time()-t0,2), L, C, R, cmdL, cmdR, label])
        f.flush()

        if touch.is_pressed:     # salir
            lm.stop(); rm.stop()
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    f.close()
    print("CSV guardado:", path)
