#!/usr/bin/env python3
# -*- coding: utf-8 -*-

print("PROGRAM START")  # <<< debug

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
import time

# ----- Audio & LEDs (con fallback) -----
try:
    from ev3dev2.sound import Sound
    sound = Sound()
    print("Sound OK")
except Exception as e:
    sound = None
    print("Sound init FAILED:", e)

try:
    from ev3dev2.led import Leds
    leds = Leds()
    print("LEDs OK")
except Exception as e:
    leds = None
    print("LED init FAILED:", e)

# ----- Motors -----
print("Init motors...")
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = rm.stop_action = 'brake'
lm.polarity = 'inversed'
rm.polarity = 'inversed'
print("Motors ready")

# ----- Sensors -----
print("Init sensors...")
L = ColorSensor(INPUT_1); L.mode = 'COL-REFLECT'
C = ColorSensor(INPUT_2); C.mode = 'COL-REFLECT'
R = ColorSensor(INPUT_3); R.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
print("Sensors ready")

def clamp(x, a=-100, b=100):
    return max(a, min(b, x))

def norm(v, w, b):
    return clamp((v - b) / max(1.0, (w - b)), 0.0, 1.0)

def wait_press_release():
    while not touch.is_pressed:
        time.sleep(0.01)
    while touch.is_pressed:
        time.sleep(0.01)

# ---------- CALIBRATION ----------
print("Calibration: WHITE step")
if leds: 
    leds.set_color('LEFT','AMBER'); leds.set_color('RIGHT','AMBER')
if sound:
    sound.play_tone(600,150); time.sleep(0.15)
else:
    print("No sound available for ping")

print("Place the SENSORS over WHITE and press the touch sensor...")
if sound:
    sound.speak("Place sensors over white and press the button")
wait_press_release()
wL = L.reflected_light_intensity
wC = C.reflected_light_intensity
wR = R.reflected_light_intensity
print("WHITE values:", wL, wC, wR)
if sound: sound.beep()

print("Now place them over BLACK and press the touch sensor...")
if sound:
    sound.speak("Now over black and press the button")
wait_press_release()
bL = L.reflected_light_intensity
bC = C.reflected_light_intensity
bR = R.reflected_light_intensity
print("BLACK values:", bL, bC, bR)
if sound: sound.beep()

# ---------- PARAMETERS ----------
BASE       = 12
MIN_BASE   = 10
Kp         = 35
SLOW_K     = 10
DT         = 0.02

SPIN       = 20
SPIN_TIME  = 0.25

last_error = 0.0

# ---------- READY TO START ----------
print("Calibrated. Ready. Press touch sensor to START.")
if leds:
    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
if sound:
    sound.speak("Calibration complete. Ready to start. Press the button to begin.")
wait_press_release()

if sound:
    for f in (800,900,1000):
        sound.play_tone(f,150)
        time.sleep(0.15)
    sound.speak("Go!")
print("Starting line following loop...")

if leds:
    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

# ---------- MAIN LOOP ----------
try:
    while True:
        nL = norm(L.reflected_light_intensity, wL, bL)
        nC = norm(C.reflected_light_intensity, wC, bC)
        nR = norm(R.reflected_light_intensity, wR, bR)

        error = (nL - nR)

        curve_factor = abs(error)
        base_now = BASE - SLOW_K * curve_factor * (0.6 + 0.4*(1.0-nC))
        base_now = max(MIN_BASE, int(base_now))

        turn = int(Kp * error)

        left_speed  = clamp(base_now - turn)
        right_speed = clamp(base_now + turn)

        if nL>0.9 and nC>0.9 and nR>0.9:
            print("Line LOST, spinning. last_error =", last_error)
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

        if touch.is_pressed:
            print("Touch pressed: STOP")
            lm.stop(); rm.stop()
            if sound:
                sound.play_tone(500,120); sound.play_tone(350,200)
            break

        last_error = error
        time.sleep(DT)

except KeyboardInterrupt:
    print("KeyboardInterrupt, exiting...")

finally:
    lm.stop(); rm.stop()
    if leds:
        leds.set_color('LEFT','RED'); leds.set_color('RIGHT','RED')
    print("PROGRAM END")
