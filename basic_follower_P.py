#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time, os, csv

# ----------------- Motores -----------------
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'
rm.stop_action = 'brake'
lm.polarity = 'inversed'
rm.polarity = 'inversed'

# ----------------- Leds & sonido -----------------
leds = Leds()
sound = Sound()

def leds_color(color):
    leds.set_color('LEFT', color)
    leds.set_color('RIGHT', color)

# ----------------- Constantes esquinas -----------------
OPPOSITE_LOCK_MS = 700   # ignore opposite side this time window
CENTER_GATE_MS   = 150   # min time with center dark to allow lane change
CORNER_PIVOT_MS  = 180   # pivot duration in corners
REV_INNER        = 16
MIN_BASE_CORNER  = 6
TIGHT_K          = 8

side_lock_until = 0.0
now = lambda: time.time()

# ----------------- Sensores -----------------
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)

# ---------- Touch helper ----------
def wait_press_release():
    """Esperar a que se presione y luego se suelte el botón."""
    while not touch.is_pressed:
        time.sleep(0.01)
    while touch.is_pressed:
        time.sleep(0.01)

# ===================== CALIBRACIÓN =====================

print("Place robot over WHITE line area and press touch...")
leds_color('GREEN')
sound.speak("Vel com locked")     # linea divertida
wait_press_release()

whiteL = csL.reflected_light_intensity
whiteC = csC.reflected_light_intensity
whiteR = csR.reflected_light_intensity

print("Now place it over BLACK line and press touch...")
leds_color('AMBER')
sound.speak("Targets designated")
wait_press_release()

blackL = csL.reflected_light_intensity
blackC = csC.reflected_light_intensity
blackR = csR.reflected_light_intensity

# ---------- Thresholds con histéresis ----------
def mid(a, b):
    return (a + b) / 2.0

thL = mid(whiteL, blackL)
thC = mid(whiteC, blackC)
thR = mid(whiteR, blackR)
HYST = 2.0

thL_on, thL_off = thL + HYST, thL - HYST
thC_on, thC_off = thC + HYST, thC - HYST
thR_on, thR_off = thR + HYST, thR - HYST

print("Thresholds:", round(thL,1), round(thC,1), round(thR,1))
print("Ready to start, press touch.")
leds_color('RED')
sound.speak("Go ahead Tac com. Press the button when you are ready to start.")
wait_press_release()

# ===================== CUENTA REGRESIVA =====================

print("Starting in...")
leds_color('GREEN')
for n, f in [(3, 600), (2, 800), (1, 1000)]:
    print(n)
    
    time.sleep(0.5)

print("Go!")
sound.speak("Go")

# ===================== CSV logging =====================
os.makedirs("/home/robot/logs", exist_ok=True)
path = "/home/robot/logs/line_edge_{0}.csv".format(time.strftime("%Y%m%d_%H%M%S"))
f = open(path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","cmdL","cmdR","state","last_side"])

# ===================== Parámetros seguidor =====================
# Sube un poco las velocidades para asegurar movimiento
BASE_BASE   = 18         # antes 12
TURN_BASE   = 26
BOOST_BASE  = 34
SEARCH_SLOW = 12
DT          = 0.02
WIDTH_FACTOR = 1.35

BASE  = BASE_BASE
TURN  = int(TURN_BASE  * WIDTH_FACTOR)
BOOST = int(BOOST_BASE * (0.9 + 0.2*WIDTH_FACTOR))
SPIN  = int(20 * WIDTH_FACTOR)

FOLLOW_LEFT = False      # False = seguir borde derecho
last_side = -1 if FOLLOW_LEFT else 1
sawL = sawC = sawR = False
t0 = time.time()
t_both_dark = 0.0
t_lost = None
center_dark_since = None
corner_until = 0.0

def with_turn_ramp(base, turn):
    return max(10, base - abs(turn)//3)

kP = 1.0 * WIDTH_FACTOR

def clamp(x, a=-100, b=100):
    return max(a, min(b, x))

# ===================== BUCLE PRINCIPAL =====================
print("Starting line follower...")
leds_color('GREEN')

try:
    while True:
        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        # ---- Histeresis booleans ----
        if not sawL: sawL = (L < thL_on)
        else:        sawL = (L < thL_off)

        if not sawC: sawC = (C < thC_on)
        else:        sawC = (C < thC_off)

        if not sawR: sawR = (R < thR_on)
        else:        sawR = (R < thR_off)

        state = "RUN"
        cmdL = cmdR = BASE
        line_detected = sawL or sawC or sawR

        # ---- Filtros: lock & center gate ----
        if not FOLLOW_LEFT and now() < side_lock_until:
            sawL = False
        if FOLLOW_LEFT and now() < side_lock_until:
            sawR = False

        if sawC:
            if center_dark_since is None:
                center_dark_since = now()
        else:
            center_dark_since = None

        # ---- Recordar último lado ----
        if FOLLOW_LEFT:
            if sawL:
                last_side = -1
            elif sawC:
                pass
            elif sawR and (now() > side_lock_until) and (center_dark_since is not None) \
                 and (now() - center_dark_since >= CENTER_GATE_MS/1000.0):
                last_side = 1
        else:
            if sawR:
                last_side = 1
            elif sawC:
                pass
            elif sawL and (now() > side_lock_until) and (center_dark_since is not None) \
                 and (now() - center_dark_since >= CENTER_GATE_MS/1000.0):
                last_side = -1

        # ---- Lógica curvas/cruces ----
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
                    cmdL = BASE; cmdR = BASE; state = "CENTER"
                elif sawR:
                    base_now = with_turn_ramp(BASE, TURN//2)
                    cmdL = clamp(base_now - TURN//2)
                    cmdR = clamp(base_now + TURN//2)
                    state = "RECOVER_L"
            else:
                # ----- Borde derecho -----
                if now() < corner_until:
                    cmdL = clamp(MIN_BASE_CORNER + BOOST)
                    cmdR = clamp(-REV_INNER)
                    state = "CORNER_R_HOLD"
                elif sawR:
                    if (not sawC) and (R < thR - TIGHT_K):
                        side_lock_until = now() + OPPOSITE_LOCK_MS/1000.0
                        corner_until    = now() + CORNER_PIVOT_MS/1000.0
                        cmdL = clamp(MIN_BASE_CORNER + BOOST)
                        cmdR = clamp(-REV_INNER)
                        state = "CORNER_R"
                    else:
                        err  = max(0.0, (thR - R))
                        turn = clamp(int(kP * err), -BOOST, BOOST)
                        if (not sawC) and (R < thR - 6):
                            turn = clamp(BOOST, -BOOST, BOOST)
                        base_now = with_turn_ramp(BASE, turn)
                        cmdL = clamp(base_now + turn)
                        cmdR = clamp(base_now - turn)
                        state = "EDGE_R"
                elif sawC:
                    cmdL = BASE; cmdR = BASE; state = "CENTER"
                elif sawL:
                    base_now = with_turn_ramp(BASE, TURN//2)
                    cmdL = clamp(base_now + TURN//2)
                    cmdR = clamp(base_now - TURN//2)
                    state = "RECOVER_R"

        else:
            # ---- Línea perdida ----
            state = "SEARCH"
            if t_lost is None:
                t_lost = time.time()
            lost_time = time.time() - t_lost

            spin = SPIN
            if last_side <= 0:
                cmdL = -spin; cmdR =  spin
            else:
                cmdL =  spin; cmdR = -spin

            if now() < side_lock_until:
                mini = SPIN
                if FOLLOW_LEFT:
                    cmdL = -mini; cmdR = mini
                else:
                    cmdL = mini;  cmdR = -mini

            if lost_time > 0.8:
                state = "SEARCH_FWD"
                cmdL = SEARCH_SLOW
                cmdR = SEARCH_SLOW
                if lost_time > 1.2:
                    t_lost = time.time()

        # ---- Ejecutar comandos ----
        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        writer.writerow([round(time.time()-t0,2), L, C, R, cmdL, cmdR, state, last_side])
        f.flush()

        # Tocar otra vez para parar
        if touch.is_pressed:
            lm.stop(); rm.stop()
            break

        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    f.close()
    leds_color('RED')
    print("CSV saved at:", path)
    sound.speak("Acknowledged H Q")
