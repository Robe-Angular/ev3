#!/usr/bin/env python3
# lf_simple3_log_hq_calib.py
#
# Seguidor de línea SIMPLE con 3 sensores:
# - Reglas básicas (LEFT / CENTER / RIGHT / LOST)
# - LENTO y estable
# - START/STOP con TouchSensor
# - LOG a CSV: t,L,C,R,cmdL,cmdR,state,last_side
# - Frases: "Acknowledged HQ", "Vel com locked", "Targets designated"
# - CALIBRACIÓN de blanco/negro por sensor (L,C,R)

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sound import Sound

import time
import csv
import os

# =====================
#  MOTORES Y SENSORES
# =====================

lm = LargeMotor(OUTPUT_B)   # motor izquierdo
rm = LargeMotor(OUTPUT_C)   # motor derecho

# Si se va hacia atrás, cambia 'inversed' a 'normal'
lm.polarity = 'inversed'
rm.polarity = 'inversed'

lm.stop_action = 'brake'
rm.stop_action = 'brake'

csL = ColorSensor(INPUT_1)
csC = ColorSensor(INPUT_2)
csR = ColorSensor(INPUT_3)

for cs in [csL, csC, csR]:
    cs.mode = 'COL-REFLECT'

touch = TouchSensor(INPUT_4)
sound = Sound()

# =====================
#  PARÁMETROS
# =====================

BASE_SPEED   = 10   # velocidad recto (bajita para estabilidad)
TURN_FAST    = 12   # rueda exterior en giro
TURN_SLOW    = 4    # rueda interior en giro
SEARCH_TURN  = 8    # giro suave cuando está perdido

DT = 0.03           # periodo del loop (segundos)

# Umbrales calibrados (se llenan en calibrate())
thrL = 35.0
thrC = 35.0
thrR = 35.0


# =====================
#  UTILIDADES
# =====================

def wait_for_touch_release():
    while touch.is_pressed:
        time.sleep(0.01)


def wait_for_touch_press():
    while not touch.is_pressed:
        time.sleep(0.01)


def sample_sensors(n=30, delay=0.01):
    """
    Lee los 3 sensores n veces y devuelve el promedio (L,C,R)
    """
    sumL = sumC = sumR = 0.0
    for _ in range(n):
        sumL += csL.reflected_light_intensity
        sumC += csC.reflected_light_intensity
        sumR += csR.reflected_light_intensity
        time.sleep(delay)
    return (sumL / n, sumC / n, sumR / n)


def calibrate():
    """
    Calibración sencilla:
      1) Sensores sobre blanco -> presionar botón
      2) Sensores sobre negro -> presionar botón
    Calcula umbrales por sensor: thr = (white + black) / 2
    """
    global thrL, thrC, thrR

    sound.speak("Calibration. Place sensors over white and press the button.")
    wait_for_touch_press()
    wait_for_touch_release()
    wL, wC, wR = sample_sensors()
    print("White L,C,R:", wL, wC, wR)

    sound.speak("Now place sensors over black line and press the button.")
    wait_for_touch_press()
    wait_for_touch_release()
    bL, bC, bR = sample_sensors()
    print("Black L,C,R:", bL, bC, bR)

    thrL = (wL + bL) / 2.0
    thrC = (wC + bC) / 2.0
    thrR = (wR + bR) / 2.0

    print("Thresholds L,C,R:", thrL, thrC, thrR)
    sound.speak("Vel com locked.")


# =====================
#  LÓGICA DEL CONTROL
# =====================

def follow_step(last_side):
    """
    Un paso del seguidor simple:
    Lee sensores y aplica reglas básicas.

    Usa umbrales calibrados thrL, thrC, thrR.

    Devuelve:
    last_side, state, vL, vR, L, C, R
    """
    L = csL.reflected_light_intensity
    C = csC.reflected_light_intensity
    R = csR.reflected_light_intensity

    L_BLACK = (L < thrL)
    C_BLACK = (C < thrC)
    R_BLACK = (R < thrR)

    state = "CENTER"
    vL = BASE_SPEED
    vR = BASE_SPEED

    # ---- Reglas sencillas ----

    if C_BLACK and not L_BLACK and not R_BLACK:
        # Línea al centro
        state = "CENTER"
        vL = BASE_SPEED
        vR = BASE_SPEED

    elif L_BLACK and not C_BLACK and not R_BLACK:
        # Sólo negro a la izquierda
        state = "LEFT"
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif R_BLACK and not C_BLACK and not L_BLACK:
        # Sólo negro a la derecha
        state = "RIGHT"
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif C_BLACK and L_BLACK and not R_BLACK:
        # Centro + izquierda (curva izq)
        state = "LEFT"
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif C_BLACK and R_BLACK and not L_BLACK:
        # Centro + derecha (curva der)
        state = "RIGHT"
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif L_BLACK and C_BLACK and R_BLACK:
        # Los tres negros -> recto
        state = "CENTER"
        vL = BASE_SPEED
        vR = BASE_SPEED

    else:
        # Todo blanco / raro -> LOST, buscar según last_side
        state = "LOST"
        if last_side < 0:
            vL = -SEARCH_TURN
            vR = SEARCH_TURN
        elif last_side > 0:
            vL = SEARCH_TURN
            vR = -SEARCH_TURN
        else:
            vL = 5
            vR = 5

    lm.on(SpeedPercent(vL))
    rm.on(SpeedPercent(vR))

    return last_side, state, vL, vR, L, C, R


def hq_intro():
    
    
    sound.speak("Targets designated.")


def countdown():
    
    time.sleep(0.3)
    for n in [3, 2, 1]:
        sound.speak(str(n))
        time.sleep(0.3)
    sound.speak("Go Ahead TACCOM.")


def make_log_writer():
    os.makedirs("logs", exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join("logs", "lf_simple3_hq_%s.csv" % ts)
    f = open(filename, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["t", "L", "C", "R", "cmdL", "cmdR", "state", "last_side"])
    return writer, f, filename


def main():
    sound.speak("Com link online.")
    calibrate()  # ----> se hace UNA vez al inicio

    while True:
        sound.speak("Press the button to start.")
        wait_for_touch_press()
        wait_for_touch_release()

        writer, f, filename = make_log_writer()
        print("Log CSV:", filename)

        hq_intro()
        countdown()

        t0 = time.time()
        last_side = 0

        print("Pulsa touch para detener.")

        try:
            while not touch.is_pressed:
                (last_side,
                 state,
                 vL, vR,
                 L, C, R) = follow_step(last_side)

                t = time.time() - t0

                writer.writerow([
                    "%.2f" % t,
                    L, C, R,
                    vL, vR,
                    state,
                    last_side
                ])

                time.sleep(DT)

        finally:
            lm.off()
            rm.off()
            f.close()
            print("Log guardado en:", filename)
            sound.speak("Acknowledged, H Q.")
            wait_for_touch_release()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        lm.off()
        rm.off()
