#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from svm_ev3 import SVMPortable
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

# --- Crear y configurar el modelo ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # carpeta de este script
svm_dir = os.path.join(BASE_DIR, "export_svm")

print("Cargando SVM desde:", svm_dir)  # opcional, para verificar en consola

svm = SVMPortable(path=svm_dir, mode="calib")

svm.set_calibration(whiteL, whiteC, whiteR, blackL, blackC, blackR)

# --- Mapeo de clases a velocidades ---
FWD = -7    # empuje hacia adelante
TURN = -15   # componente de giro

map_cmd = {
    "LEFT":   (FWD - TURN, FWD + TURN),   # = ( -2, 22 )  aprox
    "CENTER": (FWD,        FWD),          # = ( 10, 10 )
    "RIGHT":  (FWD + TURN, FWD - TURN),   # = ( 22, -2 )
}

print("Listo! Presiona para comenzar...")
wait_press_release()

# --- Loop principal ---
try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        label = svm.predict(L=L, C=C, R=R)
        cmdL, cmdR = map_cmd.get(label, (10, 10))

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        

        if touch.is_pressed:
            print("Touch presionado: saliendo.")
            break
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    lm.stop()
    rm.stop()
    print("Detenido.")
