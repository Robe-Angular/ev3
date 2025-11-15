#!/usr/bin/env python3
# lf_simple3_log_hq.py
#
# Seguidor de línea SIMPLE con 3 sensores:
# - Reglas básicas (LEFT / CENTER / RIGHT / LOST)
# - LENTO y estable
# - START/STOP con TouchSensor
# - LOG a CSV: t,L,C,R,cmdL,cmdR,state,last_side
# - Frases: "Acknowledged HQ, Vel com locked, Targets designated"
#
# Conecta:
#   Motores: B = IZQUIERDO, C = DERECHO
#   ColorSensor: L = INPUT_1, C = INPUT_2, R = INPUT_3
#   TouchSensor: INPUT_4

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

# Si se va hacia atrás, cambia 'inversed' por 'normal'
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

BASE_SPEED   = 10     # velocidad recto (bajita para estabilidad)
TURN_FAST    = 12     # rueda exterior en giro
TURN_SLOW    = 4      # rueda interior en giro
SEARCH_TURN  = 8      # al estar perdido, giro suave sobre su eje

BLACK_THRESHOLD = 35  # AJUSTA según tu pista (negro < este valor)

DT = 0.03             # periodo del loop (segundos)


# =====================
#  LÓGICA DEL CONTROL
# =====================

def follow_step(last_side):
    """
    Un paso del seguidor simple:
    Lee sensores y aplica reglas básicas.

    Devuelve:
    last_side, state, vL, vR, L, C, R
    """
    L = csL.reflected_light_intensity
    C = csC.reflected_light_intensity
    R = csR.reflected_light_intensity

    L_BLACK = (L < BLACK_THRESHOLD)
    C_BLACK = (C < BLACK_THRESHOLD)
    R_BLACK = (R < BLACK_THRESHOLD)

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
        # Centro + izquierda (ligera curva izq)
        state = "LEFT"
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif C_BLACK and R_BLACK and not L_BLACK:
        # Centro + derecha (ligera curva der)
        state = "RIGHT"
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif L_BLACK and C_BLACK and R_BLACK:
        # Los tres negros -> recto pero despacio
        state = "CENTER"
        vL = BASE_SPEED
        vR = BASE_SPEED

    else:
        # Todo blanco o raro -> LOST (buscar)
        state = "LOST"
        if last_side < 0:
            # veníamos de izquierda
            vL = -SEARCH_TURN
            vR = SEARCH_TURN
        elif last_side > 0:
            # veníamos de derecha
            vL = SEARCH_TURN
            vR = -SEARCH_TURN
        else:
            # sin historial, avanza muy lento
            vL = 5
            vR = 5

    # Aplicar comandos a los motores
    lm.on(SpeedPercent(vL))
    rm.on(SpeedPercent(vR))

    return last_side, state, vL, vR, L, C, R


def hq_intro():
    # Frases mamalonas antes de arrancar
    sound.speak("Acknowledged, H Q.")
    sound.speak("Vel com locked.")
    sound.speak("Targets designated.")


def countdown():
    sound.speak("Ready.")
    time.sleep(0.3)
    for n in [3, 2, 1]:
        sound.speak(str(n))
        time.sleep(0.3)
    sound.speak("Go.")


def wait_for_touch_release():
    while touch.is_pressed:
        time.sleep(0.01)


def make_log_writer():
    os.makedirs("logs", exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join("logs", "lf_simple3_hq_%s.csv" % ts)
    f = open(filename, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["t", "L", "C", "R", "cmdL", "cmdR", "state", "last_side"])
    return writer, f, filename


def main():
    sound.speak("Simple line follower ready.")

    while True:
        print("Pulsa el touch para comenzar...")
        while not touch.is_pressed:
            time.sleep(0.01)
        wait_for_touch_release()

        writer, f, filename = make_log_writer()
        print("Log CSV:", filename)

        # Intro tipo HQ + cuenta regresiva
        hq_intro()
        countdown()

        t0 = time.time()
        last_side = 0  # -1 = venía de izq, 1 = der, 0 = ninguno

        print("Siguiendo línea... Pulsa touch para detener.")

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

        wait_for_touch_release()
        sound.speak("Stopped.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        lm.off()
        rm.off()
