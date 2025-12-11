#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from perceptron_ev3 import PerceptronPortable
import time
import os

# --- Hardware ---
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
csL = ColorSensor(INPUT_1)
csC = ColorSensor(INPUT_2)
csR = ColorSensor(INPUT_3)
touch = TouchSensor(INPUT_4)

lm.stop_action = 'brake'
rm.stop_action = 'brake'

def wait_press_release():
    while not touch.is_pressed:
        time.sleep(0.01)
    while touch.is_pressed:
        time.sleep(0.01)

# --- Calibración ---
print("Coloca los sensores sobre BLANCO y presiona el b...")
wait_press_release()
whiteL = csL.reflected_light_intensity
whiteC = csC.reflected_light_intensity
whiteR = csR.reflected_light_intensity

print("Ahora sobre NEGRO y presiona el b...")
wait_press_release()
blackL = csL.reflected_light_intensity
blackC = csC.reflected_light_intensity
blackR = csR.reflected_light_intensity

print("BLANCOS:", whiteL, whiteC, whiteR)
print("NEGROS :", blackL, blackC, blackR)

# --- Crear y configurar el modelo ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # carpeta de este script
perc_dir = os.path.join(BASE_DIR, "export_perc")

print("Cargando Log Reg desde:", perc_dir)  # opcional, para verificar en consola
perceptron = PerceptronPortable(perc_dir, mode="calib")
perceptron.set_calibration(whiteL, whiteC, whiteR, blackL, blackC, blackR)

# FWD, TURN = 5, 20
FWD, TURN = -7, -15
map_cmd = {
    "LEFT":   (FWD - TURN, FWD + TURN),
    "CENTER": (FWD,        FWD),
    "RIGHT":  (FWD + TURN, FWD - TURN),
}

print("Listo! Presiona para comenzar...")
wait_press_release()

# --- Loop principal ---
try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        label = perceptron.predict(L=L, C=C, R=R)
        cmdL, cmdR = map_cmd.get(label, (10, 10))

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    lm.stop()
    rm.stop()
    print("Detenido.")

# FWD, TURN = 5, 20
# map_cmd = {
#     "LEFT":   (FWD - TURN, FWD + TURN),
#     "CENTER": (FWD,        FWD),
#     "RIGHT":  (FWD + TURN, FWD - TURN),
# }

