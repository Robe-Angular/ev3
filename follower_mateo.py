#!/usr/bin/env python3
"""
Seguidor de línea con curvas y pérdida de línea.
Incluye calibración automática para adaptar los umbrales
a la iluminación actual del entorno.
"""
from ev3dev2.auto import *
from time import perf_counter, sleep

# --- Conexiones ---
motor_der = LargeMotor(OUTPUT_A)
motor_izq = LargeMotor(OUTPUT_D)
ojo_der = ColorSensor('in1')
ojo_med = ColorSensor('in2')
ojo_izq = ColorSensor('in3')

ojo_izq.mode = 'COL-REFLECT'
ojo_der.mode = 'COL-REFLECT'
ojo_med.mode = 'COL-REFLECT'

# --- Variables Globales ---
apagado = False
tiempo_inicio = perf_counter()
tiempo_ejecucion = 0
f = open("data.txt", "w+")

# --- Calibración Automática ---
def calibrar():
    """Calibra blanco y negro en los tres sensores."""
    snd = Sound()
    snd.speak('Place sensors on white, then press center button.')
    btn = Button()
    while not btn.enter: sleep(0.1)
    blanco = [ojo_izq.value(), ojo_med.value(), ojo_der.value()]
    print("Lecturas en blanco:", blanco)
    snd.speak('Now place sensors on black line.')
    while btn.enter: sleep(0.1)
    while not btn.enter: sleep(0.1)
    negro = [ojo_izq.value(), ojo_med.value(), ojo_der.value()]
    print("Lecturas en negro:", negro)
    snd.speak('Calibration complete.')

    # Calcula umbrales promedio
    prom_b = sum(blanco) / 3
    prom_n = sum(negro) / 3
    rango = prom_b - prom_n

    VER_NEGRO = prom_n + 0.20 * rango
    VER_GRIS  = prom_n + 0.45 * rango
    VER_BLANCO = prom_n + 0.70 * rango

    print("Umbrales -> NEGRO:", VER_NEGRO, "GRIS:", VER_GRIS, "BLANCO:", VER_BLANCO)
    return VER_NEGRO, VER_GRIS, VER_BLANCO

VER_NEGRO, VER_GRIS, VER_BLANCO = calibrar()

# --- Velocidades ---
VEL_ALTA = 80
VEL_MEDIA = 50
VEL_BAJA = 10
VEL_INVERSA = -50

# --- Funciones ---
def anota():
    datos = (ojo_izq.value(), ojo_med.value(), ojo_der.value(), motor_izq.speed, motor_der.speed)
    print(datos)
    f.write(str(datos) + "\n")

def apagador():
    print("Apagando motores...")
    motor_izq.stop()
    motor_der.stop()

def run():
    global apagado, tiempo_ejecucion

    while not apagado and tiempo_ejecucion < 60:
        tiempo_ejecucion = perf_counter() - tiempo_inicio

        val_izq = ojo_izq.value()
        val_med = ojo_med.value()
        val_der = ojo_der.value()

        # 1️⃣ Curva 90° derecha
        if val_izq < VER_NEGRO and val_med < VER_NEGRO:
            print("Giro 90° derecha")
            motor_izq.run_forever(speed_sp=VEL_ALTA)
            motor_der.run_forever(speed_sp=VEL_INVERSA)

        # 2️⃣ Curva 90° izquierda
        elif val_der < VER_NEGRO and val_med < VER_NEGRO:
            print("Giro 90° izquierda")
            motor_izq.run_forever(speed_sp=VEL_INVERSA)
            motor_der.run_forever(speed_sp=VEL_ALTA)

        # 3️⃣ Correcciones normales
        elif val_izq < VER_NEGRO:
            print("Gira duro derecha")
            motor_izq.run_forever(speed_sp=VEL_ALTA)
            motor_der.run_forever(speed_sp=VEL_BAJA)

        elif val_der < VER_NEGRO:
            print("Gira duro izquierda")
            motor_izq.run_forever(speed_sp=VEL_BAJA)
            motor_der.run_forever(speed_sp=VEL_ALTA)

        elif val_izq < VER_GRIS:
            print("Gira derecha")
            motor_izq.run_forever(speed_sp=VEL_ALTA)
            motor_der.run_forever(speed_sp=VEL_MEDIA)

        elif val_der < VER_GRIS:
            print("Gira izquierda")
            motor_izq.run_forever(speed_sp=VEL_MEDIA)
            motor_der.run_forever(speed_sp=VEL_ALTA)

        # 4️⃣ Línea perdida
        elif val_izq > VER_BLANCO and val_med > VER_BLANCO and val_der > VER_BLANCO:
            print("¡Línea perdida! Deteniendo robot.")
            apagado = True
            break

        # 5️⃣ Avance recto
        else:
            print("OK recto")
            motor_izq.run_forever(speed_sp=VEL_ALTA)
            motor_der.run_forever(speed_sp=VEL_ALTA)

        anota()
        sleep(0.1)

# --- Ejecución principal ---
run()
apagador()
f.close()
