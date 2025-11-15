#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
import time

# ----- Motores -----
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = rm.stop_action = 'brake'
# Si “adelante” te sale al revés, descomenta:
lm.polarity = 'inversed'
rm.polarity = 'inversed'

# ----- Sensores -----
L = ColorSensor(INPUT_1); L.mode = 'COL-REFLECT'
C = ColorSensor(INPUT_2); C.mode = 'COL-REFLECT'
R = ColorSensor(INPUT_3); R.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)

def clamp(x, a=-100, b=100): 
    return max(a, min(b, x))

def norm(v, w, b):
    # Normaliza 0..1 (0=negro, 1=blanco) y evita divisiones raras
    return clamp((v - b) / max(1.0, (w - b)), 0.0, 1.0)

print("Coloca los SENSORES sobre BLANCO y pulsa el touch...")
while not touch.is_pressed: time.sleep(0.01)
while touch.is_pressed: time.sleep(0.01)
wL, wC, wR = L.reflected_light_intensity, C.reflected_light_intensity, R.reflected_light_intensity

print("Ahora sobre NEGRO y pulsa el touch...")
while not touch.is_pressed: time.sleep(0.01)
while touch.is_pressed: time.sleep(0.01)
bL, bC, bR = L.reflected_light_intensity, C.reflected_light_intensity, R.reflected_light_intensity

# ----- Parámetros FÁCILES -----
BASE   = 14   # velocidad base (bájala si aún va rápido: 12–14)
Kp     = 40   # ganancia proporcional (30–50 típico)
SLOW_K = 12   # baja base en curvas: base_now = BASE - SLOW_K*|error|
DT     = 0.02

# Recovery MUY simple si pierde línea
SPIN = 20         # giro al buscar
SPIN_TIME = 0.25  # segundos de giro antes de volver a probar

last_error = 0.0

try:
    while True:
        # Lecturas normalizadas (0=negro, 1=blanco)
        nL = norm(L.reflected_light_intensity, wL, bL)
        nC = norm(C.reflected_light_intensity, wC, bC)
        nR = norm(R.reflected_light_intensity, wR, bR)

        # -------- Control P continuo con 3 sensores --------
        # Error principal: diferencia L-R (continuo y estable)
        error = (nL - nR)   # >0 → línea a la derecha (R más oscuro) → girar a la derecha

        # Suaviza la base en curvas (si el centro está oscuro, va sobre la línea: menos penalización)
        curve_factor = abs(error)
        base_now = max(8, BASE - int(SLOW_K * curve_factor * (0.6 + 0.4*(1.0 - nC))))
        turn = int(Kp * error)

        left  = clamp(base_now - turn)
        right = clamp(base_now + turn)

        # -------- Pérdida de línea muy simple --------
        # Si TODO se ve blanco (≈1), probablemente perdió la línea → girito corto hacia donde iba corrigiendo
        if nL > 0.9 and nC > 0.9 and nR > 0.9:
            # Usa el signo del último error para decidir el lado
            if last_error >= 0:
                left, right =  SPIN, -SPIN
            else:
                left, right = -SPIN,  SPIN
            t_end = time.time() + SPIN_TIME
            while time.time() < t_end:
                lm.on(SpeedPercent(left)); rm.on(SpeedPercent(right))
                time.sleep(DT)
            # después del spin, sigue el loop normal
        else:
            lm.on(SpeedPercent(left))
            rm.on(SpeedPercent(right))

        last_error = error
        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
