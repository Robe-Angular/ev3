#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time

# ----- Motors -----
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = rm.stop_action = 'brake'
# Deja 'inversed' si físicamente los motores están al revés:
lm.polarity = 'inversed'
rm.polarity = 'inversed'

# ----- Sensors -----
L = ColorSensor(INPUT_1); L.mode = 'COL-REFLECT'
C = ColorSensor(INPUT_2); C.mode = 'COL-REFLECT'
R = ColorSensor(INPUT_3); R.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)

# ----- Audio & LEDs -----
sound = Sound()
leds = Leds()

def clamp(x, a=-100, b=100):
    return max(a, min(b, x))

def norm(v, w, b):
    # Normalize 0..1  (0=black, 1=white)
    return clamp((v - b) / max(1.0, (w - b)), 0.0, 1.0)

def wait_press_release():
    while not touch.is_pressed: 
        time.sleep(0.01)
    while touch.is_pressed: 
        time.sleep(0.01)

# ---------- CALIBRATION ----------
leds.set_color('LEFT',  'AMBER')
leds.set_color('RIGHT', 'AMBER')
sound.play_tone(600, 150); time.sleep(0.15)  # ping

print("Place the SENSORS over WHITE and press the touch sensor...")
sound.speak("Place sensors over white and press the button")
wait_press_release()
wL = L.reflected_light_intensity
wC = C.reflected_light_intensity
wR = R.reflected_light_intensity
sound.beep()

print("Now place them over BLACK and press the touch sensor...")
sound.speak("Now over black and press the button")
wait_press_release()
bL = L.reflected_light_intensity
bC = C.reflected_light_intensity
bR = R.reflected_light_intensity
sound.beep()

# ---------- EASY PARAMETERS ----------
BASE       = 12   # base speed (if too fast, try 10 or 11)
MIN_BASE   = 10   # minimum real speed (motor needs this to move)
Kp         = 35   # proportional gain (lower = smoother, higher = more aggressive)
SLOW_K     = 10   # how much we slow down in curves
DT         = 0.02

SPIN       = 20       # spin speed when line is lost
SPIN_TIME  = 0.25     # seconds spinning

last_error = 0.0

# ---------- READY TO START (no auto start) ----------
leds.set_color('LEFT',  'GREEN')
leds.set_color('RIGHT', 'GREEN')
print("Calibrated. Ready. Press touch sensor to START.")
sound.speak("Calibration complete. Ready to start. Press the button to begin.")
wait_press_release()

# Countdown 3-2-1 with beeps
for f in (800, 900, 1000):
    sound.play_tone(f, 150)
    time.sleep(0.15)

print("Starting line following loop...")
sound.speak("Go!")
leds.set_color('LEFT',  'GREEN')
leds.set_color('RIGHT', 'GREEN')

# ---------- MAIN LOOP ----------
try:
    while True:
        # Read and normalize (0=black, 1=white)
        nL = norm(L.reflected_light_intensity, wL, bL)
        nC = norm(C.reflected_light_intensity, wC, bC)
        nR = norm(R.reflected_light_intensity, wR, bR)

        # Error from left-right difference
        error = (nL - nR)  # >0 means line more on the right side

        # Base speed with slowdown in curves
        curve_factor = abs(error)
        base_now = BASE - SLOW_K * curve_factor * (0.6 + 0.4 * (1.0 - nC))
        base_now = max(MIN_BASE, int(base_now))

        turn = int(Kp * error)

        left_speed  = clamp(base_now - turn)
        right_speed = clamp(base_now + turn)

        # If everything is “white” → probably lost the line → small spin
        if nL > 0.9 and nC > 0.9 and nR > 0.9:
            if last_error >= 0:
                left_speed, right_speed =  SPIN, -SPIN
            else:
                left_speed, right_speed = -SPIN,  SPIN
            t_end = time.time() + SPIN_TIME
            while time.time() < t_end:
                lm.on(SpeedPercent(left_speed))
                rm.on(SpeedPercent(right_speed))
                time.sleep(DT)
        else:
            lm.on(SpeedPercent(left_speed))
            rm.on(SpeedPercent(right_speed))

        # STOP with touch sensor
        if touch.is_pressed:
            lm.stop(); rm.stop()
            sound.play_tone(500, 120)
            sound.play_tone(350, 200)
            break

        last_error = error
        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    leds.set_color('LEFT',  'RED')
    leds.set_color('RIGHT', 'RED')
