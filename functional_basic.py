#!/usr/bin/env python3
# lf_simple3.py
#
# Seguidor de línea sencillo con 3 sensores:
# - Usa sólo reglas básicas (izq / centro / der / perdido)
# - LENTO y estable
# - Sin estados CROSS_LOCK, CORNER, SEARCH complejo, etc.
# - Incluye cuenta regresiva con sonidos antes de arrancar.
#
# START/STOP: con el TouchSensor (puedes cambiarlo por botón del ladrillo si quieres)

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sound import Sound
import time

# =====================
#  CONFIGURACIÓN BÁSICA
# =====================

# Motores (ajusta OUTPUT_B / OUTPUT_C si los cambiás físicamente)
lm = LargeMotor(OUTPUT_B)   # motor izquierdo
rm = LargeMotor(OUTPUT_C)   # motor derecho

# IMPORTANTE: si el robot se va hacia atrás, cambia 'inversed' por 'normal'
lm.polarity = 'inversed'
rm.polarity = 'inversed'

lm.stop_action = 'brake'
rm.stop_action = 'brake'

# Sensores de color: L, C, R
csL = ColorSensor(INPUT_1)
csC = ColorSensor(INPUT_2)
csR = ColorSensor(INPUT_3)

for cs in [csL, csC, csR]:
    cs.mode = 'COL-REFLECT'

# Touch para arrancar / detener
touch = TouchSensor(INPUT_4)

# Sonido
sound = Sound()

# =====================
#  PARÁMETROS DEL CONTROL
# =====================

# Velocidades (puedes subirlas un poco si ya lo ves muy seguro)
BASE_SPEED = 12        # velocidad base cuando vamos bien centrados
TURN_FAST = 14         # rueda externa en la curva
TURN_SLOW = 4          # rueda interna en la curva
SEARCH_TURN = 10       # cuando se pierde la línea, giro suave

# Umbral de "negro" (ajusta según tu pista)
# Tip: pon el robot sobre la línea y revisa cs.reflected_light_intensity()
#      Si te da ~20-25 en negro y ~60-70 en blanco, un umbral 35 va bien.
BLACK_THRESHOLD = 35

# Tiempo de espera entre iteraciones
DT = 0.03   # 30 ms aprox


# =====================
#  LÓGICA SIMPLE DEL SEGUIDOR
# =====================

def follow_step(last_side):
    """
    Un paso de control:
    - Lee L, C, R
    - Decide velocidades de cada motor
    - Devuelve last_side actualizado
    """
    L = csL.reflected_light_intensity
    C = csC.reflected_light_intensity
    R = csR.reflected_light_intensity

    # DEBUG opcional por consola (si corres por SSH)
    # print("L={}, C={}, R={}".format(L, C, R))

    # Booleans de "estoy viendo negro"
    L_BLACK = (L < BLACK_THRESHOLD)
    C_BLACK = (C < BLACK_THRESHOLD)
    R_BLACK = (R < BLACK_THRESHOLD)

    # Inicialmente, seguir recto
    vL = BASE_SPEED
    vR = BASE_SPEED

    # ---- REGLAS SIMPLES ----
    if C_BLACK and not L_BLACK and not R_BLACK:
        # Línea al centro -> recto
        vL = BASE_SPEED
        vR = BASE_SPEED

    elif L_BLACK and not C_BLACK and not R_BLACK:
        # Sólo ve negro el sensor izquierdo -> girar a la IZQUIERDA
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif R_BLACK and not C_BLACK and not L_BLACK:
        # Sólo ve negro el sensor derecho -> girar a la DERECHA
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif C_BLACK and L_BLACK and not R_BLACK:
        # Centro + izquierda negros -> un poco a la izquierda pero avanzando
        vL = TURN_SLOW
        vR = TURN_FAST
        last_side = -1

    elif C_BLACK and R_BLACK and not L_BLACK:
        # Centro + derecha negros -> un poco a la derecha pero avanzando
        vL = TURN_FAST
        vR = TURN_SLOW
        last_side = 1

    elif L_BLACK and C_BLACK and R_BLACK:
        # Los tres negros (cruce grueso): avanza recto
        vL = BASE_SPEED
        vR = BASE_SPEED

    else:
        # TODOS BLANCOS o combinación rara: línea perdida
        # Usa last_side para decidir hacia dónde buscar
        if last_side < 0:
            # última vez estaba a la izquierda -> buscar girando izquierda
            vL = -SEARCH_TURN
            vR = SEARCH_TURN
        elif last_side > 0:
            # última vez estaba a la derecha -> buscar girando derecha
            vL = SEARCH_TURN
            vR = -SEARCH_TURN
        else:
            # al inicio aún no sabemos -> avanza muy despacio
            vL = 5
            vR = 5

    # Aplica velocidades
    lm.on(SpeedPercent(vL))
    rm.on(SpeedPercent(vR))

    return last_side


def countdown():
    """
    Cuenta regresiva con sonidos antes de arrancar.
    """
    sound.speak("Preparado")
    time.sleep(0.5)
    for n in [3, 2, 1]:
        sound.speak(str(n))
        time.sleep(0.4)
    sound.speak("Ya")


def wait_for_touch_release():
    while touch.is_pressed:
        time.sleep(0.01)


def main():
    sound.speak("Seguidor simple listo")

    while True:
        # Espera a que presiones el touch para empezar
        print("Pulsa el touch para comenzar a seguir la linea...")
        while not touch.is_pressed:
            time.sleep(0.01)
        wait_for_touch_release()

        countdown()

        # Bucle de seguimiento hasta que vuelvas a pulsar el touch
        last_side = 0
        print("Siguiendo la linea... Pulsa el touch para detener.")
        while not touch.is_pressed:
            last_side = follow_step(last_side)
            time.sleep(DT)

        # Stop
        lm.off()
        rm.off()
        wait_for_touch_release()
        sound.speak("Detenido")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        lm.off()
        rm.off()
