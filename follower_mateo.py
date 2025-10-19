#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Line follower with automatic calibration and persistent search mode.
Keeps searching indefinitely toward the last side seen when line is lost.
Press the touch sensor to stop.
Compatible with Python 3.5 (EV3).
"""

from ev3dev2.auto import *
from time import perf_counter, sleep

# --- Hardware ---
motor_der = LargeMotor(OUTPUT_A)
motor_izq = LargeMotor(OUTPUT_D)
ojo_der = ColorSensor('in1')
ojo_med = ColorSensor('in2')
ojo_izq = ColorSensor('in3')
touch = TouchSensor('in4')
snd = Sound()
leds = Leds()

for s in [ojo_izq, ojo_med, ojo_der]:
    s.mode = 'COL-REFLECT'

# --- Variables globales ---
apagado = False
tiempo_inicio = perf_counter()
tiempo_ejecucion = 0
f = open("/home/robot/logs/line_search.csv", "w+")

# --- Calibración ---
def calibrar():
    snd.speak('Place sensors on white and press the button.')
    while not touch.is_pressed: sleep(0.1)
    while touch.is_pressed: sleep(0.1)
    blanco = [ojo_izq.value(), ojo_med.value(), ojo_der.value()]
    print("White:", blanco)

    snd.speak('Now place sensors on black line and press the button.')
    while not touch.is_pressed: sleep(0.1)
    while touch.is_pressed: sleep(0.1)
    negro = [ojo_izq.value(), ojo_med.value(), ojo_der.value()]
    print("Black:", negro)

    prom_b = sum(blanco)/3
    prom_n = sum(negro)/3
    rango = prom_b - prom_n

    VER_NEGRO = prom_n + 0.20*rango
    VER_GRIS  = prom_n + 0.45*rango
    VER_BLANCO = prom_n + 0.70*rango
    snd.speak('Calibration complete.')
    print("Thresholds:", VER_NEGRO, VER_GRIS, VER_BLANCO)
    return VER_NEGRO, VER_GRIS, VER_BLANCO

VER_NEGRO, VER_GRIS, VER_BLANCO = calibrar()

# --- Velocidades ---
VEL_ALTA = 80
VEL_MEDIA = 50
VEL_BAJA = 10

# --- Función de búsqueda persistente ---
def buscar_linea(ultimo_lado):
    """Busca la línea girando hacia el último lado visto, sin límite de tiempo."""
    snd.speak('Searching line')
    leds.set_color('LEFT', 'ORANGE'); leds.set_color('RIGHT', 'ORANGE')
    while True:
        # Si se presiona el botón → abortar búsqueda
        if touch.is_pressed:
            snd.speak('Manual stop.')
            motor_izq.stop(); motor_der.stop()
            return False

        val_izq = ojo_izq.value()
        val_med = ojo_med.value()
        val_der = ojo_der.value()

        # Si cualquiera ve negro → línea encontrada
        if val_izq < VER_NEGRO or val_med < VER_NEGRO or val_der < VER_NEGRO:
            snd.speak('Line reacquired.')
            leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
            return True

        # Gira continuamente hacia el último lado donde vio línea
        if ultimo_lado == 'izq':
            motor_izq.run_forever(speed_sp=VEL_BAJA)
            motor_der.run_forever(speed_sp=VEL_ALTA)
        else:
            motor_izq.run_forever(speed_sp=VEL_ALTA)
            motor_der.run_forever(speed_sp=VEL_BAJA)

        sleep(0.05)

# --- Bucle principal ---
ultimo_lado = 'der'  # lado inicial preferido

snd.speak('Ready. Press button to start.')
while not touch.is_pressed: sleep(0.1)
while touch.is_pressed: sleep(0.1)
snd.speak('Line following started.')
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

while not apagado:
    if touch.is_pressed:  # botón para terminar manualmente
        snd.speak('Program stopped.')
        apagado = True
        break

    val_izq = ojo_izq.value()
    val_med = ojo_med.value()
    val_der = ojo_der.value()

    # --- Reglas principales ---
    if val_izq < VER_NEGRO and val_med < VER_NEGRO:
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=-VEL_BAJA)
        ultimo_lado = 'der'

    elif val_der < VER_NEGRO and val_med < VER_NEGRO:
        motor_izq.run_forever(speed_sp=-VEL_BAJA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
        ultimo_lado = 'izq'

    elif val_izq < VER_NEGRO:
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_BAJA)
        ultimo_lado = 'der'

    elif val_der < VER_NEGRO:
        motor_izq.run_forever(speed_sp=VEL_BAJA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
        ultimo_lado = 'izq'

    elif val_izq < VER_GRIS:
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_MEDIA)
        ultimo_lado = 'der'

    elif val_der < VER_GRIS:
        motor_izq.run_forever(speed_sp=VEL_MEDIA)
        motor_der.run_forever(speed_sp=VEL_ALTA)
        ultimo_lado = 'izq'

    elif val_izq > VER_BLANCO and val_med > VER_BLANCO and val_der > VER_BLANCO:
        print("Line lost → searching", ultimo_lado)
        motor_izq.stop(); motor_der.stop()
        if not buscar_linea(ultimo_lado):
            apagado = True
            break

    else:
        motor_izq.run_forever(speed_sp=VEL_ALTA)
        motor_der.run_forever(speed_sp=VEL_ALTA)

    f.write("{},{},{}\n".format(val_izq, val_med, val_der))
    sleep(0.1)

motor_izq.stop(); motor_der.stop()
snd.speak('Acknowledged H.Q.')
f.close()
