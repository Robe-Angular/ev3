#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
import time, os, csv

lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'
rm.stop_action = 'brake'

TIGHT_K = 8 
MIN_BASE_CORNER = 8
REV_INNER = 10
OPPOSITE_LOCK_MS = 350

side_lock_until = 0.0  # (state) tiempo hasta el que ignoramos el lado opuesto
now = lambda: time.time()

csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
# ✨ Arregla que "adelante" esté yendo "atrás"
lm.polarity = 'inversed'
rm.polarity = 'inversed'
sound = Sound()
# ---------- Calibración rápida ----------
def wait_press_release():
    while not touch.is_pressed: time.sleep(0.01)
    while touch.is_pressed: time.sleep(0.01)

print("Coloca sobre BLANCO y pulsa...")
# sound.speak("VelCom locked")   # Alistando robot
wait_press_release()
whiteL, whiteC, whiteR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

print("Ahora sobre NEGRO y pulsa...")

wait_press_release()
# sound.speak("Targets Designated")   #Alistando robot
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
# sound.speak("Go ahead TACCOM")   #Alistando robot
# ---------- CSV ----------
os.makedirs("/home/robot/logs", exist_ok=True)
path = "/home/robot/logs/line_edge_{0}.csv".format(time.strftime("%Y%m%d_%H%M%S"))
f = open(path, "w", newline="")
import csv
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","state","last_side"])

# ---------- Parámetros ----------
BASE_BASE  = 18         # velocidad base
TURN_BASE  = 18         # giro normal
BOOST_BASE = 28         # pulso fuerte en curva cerrada
SEARCH_SLOW = 10   # velocidad en search
DT    = 0.02       # ciclo (s)
WIDTH_FACTOR = 1.35


BASE  = BASE_BASE
TURN  = int(TURN_BASE  * WIDTH_FACTOR)
BOOST = int(BOOST_BASE * (0.9 + 0.2*WIDTH_FACTOR))  # un poco menos agresivo que TURN
SPIN  = int(14 * WIDTH_FACTOR)  # para SEARCH



FOLLOW_LEFT = False     # True: sigue borde izquierdo | False: derecho
last_side = -1 if FOLLOW_LEFT else 1   # -1=izq, +1=der
sawL = sawC = sawR = False
t0 = time.time()
t_both_dark = 0.0
t_lost = None

# Rampa: baja base cuando el giro es grande
def with_turn_ramp(base, turn):
    return max(10, base - abs(turn)//3)

# Ganancia proporcional (ajusta 0.8–1.6 * WIDTH_FACTOR si zigzaguea)
kP = 1.0 * WIDTH_FACTOR
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

        line_detected = sawL or sawC or sawR

        # Memoriza último lado visto (prioriza borde elegido)
        if FOLLOW_LEFT:
            if sawL:
                last_side = -1
            elif sawC:
                pass
            elif sawR and now() > side_lock_until:   # ignora lado opuesto durante lock
                last_side = 1
        else:
            if sawR:
                last_side = 1
            elif sawC:
                pass
            elif sawL and now() > side_lock_until:   # ignora lado opuesto durante lock
                last_side = -1

        # ¿Curva/cruceta? (dos sensores ven negro)
        if (sawL and sawR) or (sawL and sawC and not sawR) or (sawR and sawC and not sawL):
            t_both_dark += DT
            bias = 6 if FOLLOW_LEFT else -6
            base_now = with_turn_ramp(BASE, bias)
            cmdL = clamp(base_now - bias)
            cmdR = clamp(base_now + bias)
            state = "CROSS_LOCK"
            t_lost = None

        elif line_detected:
            t_both_dark = 0.0
            t_lost = None

            if FOLLOW_LEFT:
                # ======= Borde izquierdo =======
                if sawL:
                    err  = max(0.0, (thL - L))
                    turn = clamp(int(kP * err), -BOOST, BOOST)
                    if (not sawC) and (L < thL - 6):
                        turn = clamp(BOOST, -BOOST, BOOST)
                    base_now = with_turn_ramp(BASE, turn)
                    cmdL = clamp(base_now - turn)
                    cmdR = clamp(base_now + turn)
                    state = "EDGE_L"
                elif sawC:
                    cmdL = BASE; cmdR = BASE
                    state = "CENTER"
                elif sawR:
                    base_now = with_turn_ramp(BASE, TURN//2)
                    cmdL = clamp(base_now - TURN//2)
                    cmdR = clamp(base_now + TURN//2)
                    state = "RECOVER_L"

            else:
                # ======= Borde derecho =======
                if sawR:
                    # --- ESQUINA CERRADA DERECHO ---
                    if (not sawC) and (R < thR - TIGHT_K):
                        side_lock_until = now() + OPPOSITE_LOCK_MS/1000.0  # bloquea lado opuesto un rato
                        # Pivot controlado: rueda externa (L) acelera, interna (R) frena/retrocede
                        cmdL = clamp(MIN_BASE_CORNER + BOOST)
                        cmdR = clamp(-REV_INNER)
                        state = "CORNER_R"
                    else:
                        # --- Control proporcional normal EDGE_R ---
                        err  = max(0.0, (thR - R))
                        turn = clamp(int(kP * err), -BOOST, BOOST)
                        if (not sawC) and (R < thR - 6):
                            turn = clamp(BOOST, -BOOST, BOOST)
                        base_now = with_turn_ramp(BASE, turn)
                        cmdL = clamp(base_now + turn)
                        cmdR = clamp(base_now - turn)
                        state = "EDGE_R"
                elif sawC:
                    cmdL = BASE; cmdR = BASE
                    state = "CENTER"
                elif sawL:
                    base_now = with_turn_ramp(BASE, TURN//2)
                    cmdL = clamp(base_now + TURN//2)
                    cmdR = clamp(base_now - TURN//2)
                    state = "RECOVER_R"

        else:
            # Línea perdida: busca hacia donde la viste por última vez
            state = "SEARCH"
            if t_lost is None: 
                t_lost = time.time()
            lost_time = time.time() - t_lost

            spin = SPIN
            if last_side <= 0:
                cmdL = -spin; cmdR =  spin
            else:
                cmdL =  spin; cmdR = -spin

            # Anti-salto si el lock sigue activo: mini-pivot hacia tu borde
            if now() < side_lock_until:
                mini = SPIN
                if FOLLOW_LEFT:
                    cmdL = -mini; cmdR = mini
                else:
                    cmdL = mini;  cmdR = -mini

            # Peinado
            if lost_time > 0.8:
                state = "SEARCH_FWD"
                cmdL = SEARCH_SLOW
                cmdR = SEARCH_SLOW
                if lost_time > 1.2:
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
    # sound.speak("Acknowledged H.Q.")   #Alistando robot
    f.close()
    print("CSV guardado:", path)
