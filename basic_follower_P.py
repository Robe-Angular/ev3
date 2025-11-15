#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time

# ----- Motores -----
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = rm.stop_action = 'brake'
# Si “adelante” te sale al revés, deja estos en 'inversed'; si no, bórralos.
lm.polarity = 'inversed'
rm.polarity = 'inversed'

# ----- Sensores -----
L = ColorSensor(INPUT_1); L.mode = 'COL-REFLECT'
C = ColorSensor(INPUT_2); C.mode = 'COL-REFLECT'
R = ColorSensor(INPUT_3); R.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)

# ----- Audio y LEDs -----
sound = Sound()
leds = Leds()

def clamp(x, a=-100, b=100):
    return max(a, min(b, x))

def norm(v, w, b):
    # Normaliza 0..1 (0=negro, 1=blanco) y evita divisiones raras
    return clamp((v - b) / max(1.0, (w - b)), 0.0, 1.0)

def wait_press_release():
    while not touch.is_pressed: time.sleep(0.01)
    while touch.is_pressed: time.sleep(0.01)

# ---------- CALIBRACIÓN ----------
leds.set_color('LEFT',  'AMBER'); leds.set_color('RIGHT', 'AMBER')
sound.play_tone(600, 150); time.sleep(0.15)  # ping de inicio

print("Coloca los SENSORES sobre BLANCO y pulsa el touch...")
sound.speak("Coloca sensores sobre blanco y presiona el botón")
wait_press_release()
wL, wC, wR = L.reflected_light_intensity, C.reflected_light_intensity, R.reflected_light_intensity
sound.beep()

print("Ahora sobre NEGRO y pulsa el touch...")
sound.speak("Ahora sobre negro y presiona el botón")
wait_press_release()
bL, bC, bR = L.reflected_light_intensity, C.reflected_light_intensity, R.reflected_light_intensity
sound.beep()

# ----- Parámetros FÁCILES -----
BASE   = 5   # bájala si aún va rápido (10–14)
Kp     = 38   # 30–50 típico
SLOW_K = 12   # suaviza base en curva
DT     = 0.02

SPIN = 18         # giro al buscar
SPIN_TIME = 0.25  # s

last_error = 0.0

# ---------- CONFIRMAR INICIO (no arranca solo) ----------
leds.set_color('LEFT',  'GREEN'); leds.set_color('RIGHT', 'GREEN')
print("Calibrado. Listo. Pulsa el touch para INICIAR.")
sound.speak("Calibrado. Listo para iniciar. Presiona el botón para empezar.")
wait_press_release()

# Countdown 3-2-1 con beeps
for f in (800, 900, 1000):
    sound.play_tone(f, 150)
    time.sleep(0.15)

leds.set_color('LEFT',  'GREEN'); leds.set_color('RIGHT', 'GREEN')
leds.set(Leds.LEFT,  brightness_pct=1.0, color='GREEN')
leds.set(Leds.RIGHT, brightness_pct=1.0, color='GREEN')

# ---------- LOOP PRINCIPAL ----------
try:
    while True:
        # LED parpadeo suave para indicar "corriendo"
        leds.set_color('LEFT',  'GREEN')
        leds.set_color('RIGHT', 'GREEN')

        # Lecturas normalizadas (0=negro, 1=blanco)
        nL = norm(L.reflected_light_intensity, wL, bL)
        nC = norm(C.reflected_light_intensity, wC, bC)
        nR = norm(R.reflected_light_intensity, wR, bR)

        # Error principal: diferencia L-R
        error = (nL - nR)  # >0 → línea hacia derecha → girar derecha

        # Baja base en curvas; menos penalización si centro está oscuro
        curve_factor = abs(error)
        base_now = max(8, BASE - int(SLOW_K * curve_factor * (0.6 + 0.4*(1.0 - nC))))
        turn = int(Kp * error)

        left  = clamp(base_now - turn)
        right = clamp(base_now + turn)

        # Pérdida de línea (todo claro) → giro corto hacia el último lado
        if nL > 0.9 and nC > 0.9 and nR > 0.9:
            if last_error >= 0:
                left, right =  SPIN, -SPIN
            else:
                left, right = -SPIN,  SPIN
            t_end = time.time() + SPIN_TIME
            while time.time() < t_end:
                lm.on(SpeedPercent(left)); rm.on(SpeedPercent(right))
                time.sleep(DT)
        else:
            lm.on(SpeedPercent(left))
            rm.on(SpeedPercent(right))

        # STOP inmediato con touch
        if touch.is_pressed:
            lm.stop(); rm.stop()
            sound.play_tone(500, 120)
            sound.play_tone(350, 200)  # “bajada” de stop
            break

        last_error = error
        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    leds.set_color('LEFT',  'RED'); leds.set_color('RIGHT', 'RED')
