#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Requisitos: ev3dev2 (EV3 con Debian/ev3dev)
# sudo apt-get install python3-ev3dev2

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor, GyroSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.button import Button
import time, csv, os, math

# ---------- CONFIGURA TUS PUERTOS AQUÍ ----------
LEFT_MOTOR_PORT  = OUTPUT_B
RIGHT_MOTOR_PORT = OUTPUT_C

LEFT_COLOR_PORT  = INPUT_1
RIGHT_COLOR_PORT = INPUT_2

TOUCH_PORT       = INPUT_3          # botón rojo (Touch Sensor)
USE_GYRO        = True
GYRO_PORT        = INPUT_4
# ------------------------------------------------

# Parámetros del seguidor (para que el dataset ya tenga "acciones")
BASE_SPEED = 25          # % (0-100). Sube/baja según tu pista
KP         = 0.9         # Ganancia proporcional sobre el error (L-R)
DT         = 0.02        # periodo de muestreo (s) ~50 Hz

# Archivo de salida
ts   = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
os.makedirs(log_dir, exist_ok=True)
csv_path = os.path.join(log_dir, f"ev3_log_{ts}.csv")

# ----- Instancias de hardware -----
lm = LargeMotor(LEFT_MOTOR_PORT)
rm = LargeMotor(RIGHT_MOTOR_PORT)

csL = ColorSensor(LEFT_COLOR_PORT)
csR = ColorSensor(RIGHT_COLOR_PORT)
csL.mode = 'COL-REFLECT'    # reflectancia (0-100)
csR.mode = 'COL-REFLECT'

touch = TouchSensor(TOUCH_PORT)
gyro  = GyroSensor(GYRO_PORT) if (USE_GYRO) else None
if gyro:
    gyro.mode = 'GYRO-ANG'   # ángulo; cambia a GYRO-RATE si quieres velocidad

leds = Leds()
snd  = Sound()
btn  = Button()

# ----- Helpers -----
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wait_press_release_touch():
    # Espera presionar y soltar el touch (evita rebotes)
    while not touch.is_pressed:
        time.sleep(0.01)
    while touch.is_pressed:
        time.sleep(0.01)

# ----- Calibración rápida (opcional) -----
# Presiona el botón una vez apuntando a blanco y otra a negro para promediar.
def quick_calibration():
    snd.speak('Calibracion blanca. Presiona.')
    wait_press_release_touch()
    whiteL, whiteR = csL.reflected_light_intensity, csR.reflected_light_intensity

    snd.speak('Calibracion negra. Presiona.')
    wait_press_release_touch()
    blackL, blackR = csL.reflected_light_intensity, csR.reflected_light_intensity

    # Escala lineal a 0..100
    def scaler(x, lo, hi):
        return lambda v: clamp(100 * (v - lo) / max(1, hi - lo), 0, 100)

    # Asegura que black < white (si no, intercambia)
    loL, hiL = (blackL, whiteL) if blackL < whiteL else (whiteL, blackL)
    loR, hiR = (blackR, whiteR) if blackR < whiteR else (whiteR, blackR)

    return scaler(csL, loL, hiL), scaler(csR, loR, hiR)

# Si no quieres calibrar, set a None y usamos valores crudos 0..100
scaleL = None
scaleR = None
# Descomenta para activar calibración guiada:
# scaleL, scaleR = quick_calibration()

# ---------- Espera para INICIAR ----------
snd.speak('Presiona para iniciar')
leds.set_color('LEFT',  'RED')
leds.set_color('RIGHT', 'RED')
wait_press_release_touch()

running = True
leds.set_color('LEFT',  'GREEN')
leds.set_color('RIGHT', 'GREEN')
snd.speak('Inicio')

# ---------- CSV ----------
with open(csv_path, 'w', newline='') as f:
    writer = csv.writer(f)
    header = [
        't', 'L_reflect', 'R_reflect',
        'gyro_deg', 'gyro_rate',
        'L_speed_cmd', 'R_speed_cmd',     # comandos (SpeedPercent)
        'L_duty', 'R_duty',               # esfuerzo real %
        'L_pos', 'R_pos'                  # tics del encoder
    ]
    writer.writerow(header)

    t0 = time.time()
    next_t = t0

    # Zeros iniciales
    last_angle = gyro.angle if gyro else 0
    last_time  = t0

    try:
        while running:
            # Toggle RUN/PAUSE con el touch
            if touch.is_pressed:
                snd.beep()
                # pausa motores
                lm.stop()
                rm.stop()
                # Espera a que se suelte y vuelva a presionar para continuar
                leds.set_color('LEFT',  'ORANGE')
                leds.set_color('RIGHT', 'ORANGE')
                wait_press_release_touch()
                # segundo press = continuar / o salir si BACK
                snd.beep()
                leds.set_color('LEFT',  'GREEN')
                leds.set_color('RIGHT', 'GREEN')

            # Salida de emergencia con botón BACK del brick
            if btn.backspace:
                snd.speak('Stop')
                break

            # Mantener frecuencia DT
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += DT
            now = time.time()

            # Lecturas
            rawL = csL.reflected_light_intensity
            rawR = csR.reflected_light_intensity
            L = clamp(scaleL.value(rawL) if scaleL else rawL, 0, 100) if hasattr(scaleL, 'value') else (scaleL(rawL) if scaleL else rawL)
            R = clamp(scaleR.value(rawR) if scaleR else rawR, 0, 100) if hasattr(scaleR, 'value') else (scaleR(rawR) if scaleR else rawR)

            gdeg, grate = (None, None)
            if gyro:
                gnow   = gyro.angle
                dtg    = max(1e-3, now - last_time)
                grate  = (gnow - last_angle) / dtg
                gdeg   = gnow
                last_angle = gnow
                last_time  = now

            # Control P (para que haya "acciones" en el dataset)
            error = (L - R)               # positivo = más luz a la izquierda
            turn  = KP * error
            cmdL  = clamp(BASE_SPEED - turn, -100, 100)
            cmdR  = clamp(BASE_SPEED + turn, -100, 100)

            lm.on(SpeedPercent(cmdL))
            rm.on(SpeedPercent(cmdR))

            # Motores: posición y duty (esfuerzo actual)
            l_pos = lm.position
            r_pos = rm.position
            l_dut = lm.duty_cycle_sp if hasattr(lm, 'duty_cycle_sp') else None
            r_dut = rm.duty_cycle_sp if hasattr(rm, 'duty_cycle_sp') else None

            # Log
            writer.writerow([
                round(now - t0, 4),
                int(L), int(R),
                None if gdeg is None else int(gdeg),
                None if grate is None else round(grate, 2),
                round(cmdL, 1), round(cmdR, 1),
                l_dut, r_dut, l_pos, r_pos
            ])
            f.flush()

    finally:
        lm.stop()
        rm.stop()
        leds.all_off()
        snd.speak('Log guardado')
        print("CSV:", csv_path)
