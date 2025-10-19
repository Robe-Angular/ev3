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

print("Ahora sobre NEGRO y pulsa...")
wait_press_release()
blackL, blackC, blackR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

# Umbral base y márgenes de histeresis
def mid(a,b): return (a+b)/2
thL = mid(whiteL, blackL); thC = mid(whiteC, blackC); thR = mid(whiteR, blackR)
HYST = 2.0  # súbele si parpadea (1–5)

thL_on, thL_off = thL + HYST, thL - HYST
thC_on, thC_off = thC + HYST, thC - HYST
thR_on, thR_off = thR + HYST, thR - HYST

print("Umbrales:", round(thL,1), round(thC,1), round(thR,1))
print("Pulsa otra vez para empezar.")
wait_press_release()

# ---------- CSV ----------
os.makedirs("/home/robot/logs", exist_ok=True)
path = "/home/robot/logs/line_edge_{0}.csv".format(time.strftime("%Y%m%d_%H%M%S"))
f = open(path, "w", newline="")
import csv
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","state","last_side"])

# ---------- Parámetros ----------
BASE  = 18         # velocidad base
TURN  = 18         # giro normal
BOOST = 28         # pulso fuerte en curva cerrada
SEARCH_SLOW = 10   # velocidad en búsqueda
DT    = 0.02       # ciclo (s)

FOLLOW_LEFT = True     # True: sigue borde izquierdo | False: derecho
last_side = -1 if FOLLOW_LEFT else 1   # -1=izq, +1=der
sawL = sawC = sawR = False
t0 = time.time()
t_both_dark = 0.0
t_lost = None

def clamp(x, a=-100, b=100): return max(a, min(b, x))

try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        # Booleans con histeresis
        if not sawL: sawL = (L < thL_on)
        else:        sawL = (L < thL_off)

        if not sawC: sawC = (C < thC_on)
        else:        sawC = (C < thC_off)

        if not sawR: sawR = (R < thR_on)
        else:        sawR = (R < thR_off)

        state = "RUN"
        cmdL = cmdR = BASE

        # Preferimos seguir un borde fijo
        line_detected = sawL or sawC or sawR

        # Memoriza último lado visto (prioriza borde elegido)
        if FOLLOW_LEFT:
            if sawL: last_side = -1
            elif sawC: pass
            elif sawR: last_side = 1
        else:
            if sawR: last_side = 1
            elif sawC: pass
            elif sawL: last_side = -1

        # ¿Curva/cruceta? (dos sensores ven negro)
        if (sawL and sawR) or (sawL and sawC and not sawR) or (sawR and sawC and not sawL):
            # Evita “voltearte”: mantén avance y un sesgo suave hacia el borde elegido
            t_both_dark += DT
            bias = 6 if FOLLOW_LEFT else -6
            cmdL = clamp(BASE - bias)
            cmdR = clamp(BASE + bias)
            state = "CROSS_LOCK"
            t_lost = None

        elif line_detected:
            t_both_dark = 0.0
            t_lost = None
            # Control por borde elegido con pulso en esquina
            if FOLLOW_LEFT:
                if sawL:  # borde bajo el sensor L
                    # error ~ cuánto de negro ve L vs C, usa giro fuerte en esquina
                    turn = BOOST if (not sawC and L < thL - 6) else TURN
                    cmdL = clamp(BASE - turn)
                    cmdR = clamp(BASE + turn)
                    state = "EDGE_L"
                elif sawC:  # centro negro => recto
                    cmdL = BASE; cmdR = BASE
                    state = "CENTER"
                elif sawR:  # línea se movió a la derecha: corrige suave hacia izquierda
                    cmdL = clamp(BASE - TURN//2)
                    cmdR = clamp(BASE + TURN//2)
                    state = "RECOVER_L"
            else:
                if sawR:
                    turn = BOOST if (not sawC and R < thR - 6) else TURN
                    cmdL = clamp(BASE + turn)
                    cmdR = clamp(BASE - turn)
                    state = "EDGE_R"
                elif sawC:
                    cmdL = BASE; cmdR = BASE
                    state = "CENTER"
                elif sawL:
                    cmdL = clamp(BASE + TURN//2)
                    cmdR = clamp(BASE - TURN//2)
                    state = "RECOVER_R"

        else:
            # Línea perdida: busca hacia donde la viste por última vez
            state = "SEARCH"
            if t_lost is None: t_lost = time.time()
            lost_time = time.time() - t_lost

            # Pequeña rotación hacia last_side (no te des media vuelta)
            spin = 14
            if last_side <= 0:   # izquierda
                cmdL = -spin
                cmdR =  spin
            else:                # derecha
                cmdL =  spin
                cmdR = -spin

            # Si pasa mucho sin hallarla, avanza un poco y repite (patrón peinado)
            if lost_time > 0.8:
                state = "SEARCH_FWD"
                cmdL = SEARCH_SLOW
                cmdR = SEARCH_SLOW
                if lost_time > 1.2:
                    # reinicia ventana
                    t_lost = time.time()

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        writer.writerow([round(time.time()-t0,2), L, C, R, cmdL, cmdR, state, last_side])
        f.flush()

        if touch.is_pressed:
            lm.stop(); rm.stop()
            break

        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    f.close()
    print("CSV guardado:", path)
