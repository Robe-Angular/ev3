#!/usr/bin/env python3
# lf_simple3_log.py
#
# Seguidor de línea sencillo con 3 sensores:
# - Reglas básicas (izq / centro / der / perdido)
# - LENTO y estable
# - Sin FSM loca (ni CROSS_LOCK, ni EDGE, etc.)
# - Cuenta regresiva con sonidos
# - LOG a CSV: t,L,C,R,cmdL,cmdR,state,last_side
#
# START/STOP con TouchSensor en INPUT_4

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sound import Sound

import time
import csv
import os

# =====================
#  CONFIGURACIÓN BÁSICA
# =====================

# Motores (ajusta OUTPUT_B / OUTPUT_C si cambiás cables)
lm = LargeMotor(OUTPUT_B)   # motor izquierdo
rm = LargeMotor(OUTPUT_C)   # motor derecho

# Si el robot se va para atrás, cambia 'inversed' -> 'normal'
lm.polarity = 'inversed'
rm.polarity = 'inversed'

lm.stop_action = 'brake'
rm.stop_action = 'brake'

# Sensores de color L, C, R
csL = ColorSensor(INPUT_1)
csC = ColorSensor(INPUT_2)
csR = ColorSensor(INPUT_3)

for cs in [csL, csC, csR]:
    cs.mode = 'COL-REFLECT'

# Touch para arrancar/detener
touch = TouchSensor(INPUT_4)

# Sonido
sound = Sound()

# =====================
#  PARÁMETROS DEL CONTROL
# =====================

BASE_SPEED = 12        # velocidad recto
TURN_FAST = 14        # rueda externa
TURN_SLOW = 4         # rueda interna
SEARCH_TURN = 10      # cuando se pierde la línea

# Umbral de negro: ajústalo según tu pista
BLACK_THRESHOLD = 35

# Periodo del loop (segundos)
DT = 0.03


# =====================
#  LÓGICA SIMPLE DEL SEGUIDOR
# =====================

def follow_step(last_side):
    """
    Un paso de control:
    - Lee L, C, R
    - Decide velocidades de cada motor
    - Devuelve: last_side, state, vL, vR, L, C, R
    """
    L = csL.reflected_light_intensity
    C = csC.reflected_light_intensity
    R = csR.reflected_light_intensity

    # Flags de negro
    L_BLACK = (L < BLACK_THRESHOLD)
    C_BLACK = (C < BLACK_THRESHOLD)
    R_BLACK = (R < BLACK_THRESHOLD)

    # Por default: avanzar recto
    vL = BASE_SPEED
    vR = BASE_SPEED
    state = "CENTER"

    # ---- REGLAS SIMPLES ----
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
        # Centro + izquierda
        state = "LEFT"
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif C_BLACK and R_BLACK and not L_BLACK:
        # Centro + derecha
        state = "RIGHT"
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif L_BLACK and C_BLACK and R_BLACK:
        # Los tres negros
        state = "CENTER"
        vL = BASE_SPEED
        vR = BASE_SPEED

    else:
        # Perdido (todo blanco o raro)
        state = "LOST"
        if last_side < 0:
            # Buscar girando hacia la izquierda
            vL = -SEARCH_TURN
            vR = SEARCH_TURN
        elif last_side > 0:
            # Buscar girando hacia la derecha
            vL = SEARCH_TURN
            vR = -SEARCH_TURN
        else:
            # Aún no sabemos, avanza despacio
            vL = 5
            vR = 5

    # Aplicar velocidades
    lm.on(SpeedPercent(vL))
    rm.on(SpeedPercent(vR))

    return last_side, state, vL, vR, L, C, R


def countdown():
    sound.speak("Preparado")
    time.sleep(0.5)
    for n in [3, 2, 1]:
        sound.speak(str(n))
        time.sleep(0.4)
    sound.speak("Ya")


def wait_for_touch_release():
    while touch.is_pressed:
        time.sleep(0.01)


def make_log_writer():
    """
    Crea carpeta logs/ y abre un CSV nuevo con timestamp.
    Devuelve (csv_writer, file_handle) para que el main lo use.
    """
    os.makedirs("logs", exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join("logs", "lf_simple_%s.csv" % ts)

    f = open(filename, "w", newline="")
    writer = csv.writer(f)
    # Header compatible con lo que me mandas:
    writer.writerow(["t", "L", "C", "R", "cmdL", "cmdR", "state", "last_side"])
    return writer, f, filename


def main():
    sound.speak("Seguidor simple con log listo")

    while True:
        print("Pulsa el touch para comenzar a seguir la linea...")
        while not touch.is_pressed:
            time.sleep(0.01)
        wait_for_touch_release()

        # Crear nuevo log por corrida
        writer, f, filename = make_log_writer()
        print("Log CSV:", filename)

        countdown()

        last_side = 0
        t0 = time.time()
        print("Siguiendo linea... Pulsa touch para detener.")

        try:
            while not touch.is_pressed:
                last_side, state, vL, vR, L, C, R = follow_step(last_side)
                t = time.time() - t0

                # Guardar fila en CSV
                writer.writerow([
                    "%.2f" % t,
                    L, C, R,
                    vL, vR,
                    state,
                    last_side
                ])

                time.sleep(DT)
        finally:
            # Siempre asegurarnos de cerrar archivo y parar motores
            lm.off()
            rm.off()
            f.close()
            print("Log guardado en:", filename)

        wait_for_touch_release()
        sound.speak("Detenido")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        lm.off()
        rm.off()
