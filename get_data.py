#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time, csv, os

# ---------- Configuración ----------
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)

csL = ColorSensor(INPUT_1)
csC = ColorSensor(INPUT_2)
csR = ColorSensor(INPUT_3)
touch = TouchSensor(INPUT_4)

csL.mode = 'COL-REFLECT'
csC.mode = 'COL-REFLECT'
csR.mode = 'COL-REFLECT'

leds = Leds()
snd = Sound()

BASE_SPEED = 25
KP = 0.8
DT = 0.05

# ---------- Archivo CSV ----------
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
csv_path = os.path.join(log_dir, "ev3_log_" + str(ts) + ".csv")

f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(['t', 'L', 'C', 'R', 'cmdL', 'cmdR'])

snd.speak('Presiona para iniciar')
while not touch.is_pressed:
    time.sleep(0.01)
while touch.is_pressed:
    time.sleep(0.01)

snd.speak('Inicio')
leds.set_color('LEFT', 'GREEN')
leds.set_color('RIGHT', 'GREEN')

t0 = time.time()

try:
    while True:
        if touch.is_pressed:
            snd.beep()
            break

        L = csL.reflected_light_intensity
        C = csC.reflected_light_intensity
        R = csR.reflected_light_intensity

        # Cálculo de error (más avanzado)
        # ponderamos la posición de la línea entre los 3 sensores
        # idealmente el centro (C) debe estar en la línea (bajo valor)
        error = (L - R) + 0.5 * (C - ((L + R) / 2.0))

        turn = KP * error
        cmdL = BASE_SPEED - turn
        cmdR = BASE_SPEED + turn

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        writer.writerow([
            round(time.time() - t0, 3),
            L, C, R,
            round(cmdL, 1), round(cmdR, 1)
        ])
        f.flush()

        time.sleep(DT)

except KeyboardInterrupt:
    pass

finally:
    lm.stop()
    rm.stop()
    f.close()
    leds.all_off()
    snd.speak('Log guardado')
    print("CSV guardado en:", csv_path)
