#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.led import Leds
import os, time, csv

# --- Hardware ---
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'
rm.stop_action = 'brake'
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
leds = Leds()

# --- Utils ---
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wait_press_release(t):
    while not t.is_pressed: time.sleep(0.01)
    while t.is_pressed:     time.sleep(0.01)

def avg(sensor, n=8, dt=0.01):
    s = 0.0
    for _ in range(n):
        s += sensor.reflected_light_intensity
        time.sleep(dt)
    return s / n

def norm(v, lo, hi):
    d = float(hi - lo)
    if d < 1.0: d = 1.0
    x = (v - lo) / d
    if x < 0.0: x = 0.0
    if x > 1.0: x = 1.0
    return x

def apply_deadzone(cmd, dz=12.0):
    if 0 < cmd < dz:   return dz
    if -dz < cmd < 0:  return -dz
    return cmd

# --- Calibración simple: blanco y negro para L/R ---
def calibrate_lr():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    print("Pon AMBOS sensores en BLANCO y presiona el touch...")
    wait_press_release(touch)
    wL = avg(csL); wR = avg(csR)

    print("Pon AMBOS sensores en NEGRO y presiona el touch...")
    wait_press_release(touch)
    bL = avg(csL); bR = avg(csR)

    # Asegura blanco>negro
    if wL <= bL: wL, bL = bL, wL
    if wR <= bR: wR, bR = bR, wR

    # Sep mínima
    if abs(wL-bL) < 5: wL = bL + 5
    if abs(wR-bR) < 5: wR = bR + 5

    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    print("Calib OK. L: {0:.1f}/{1:.1f}  R: {2:.1f}/{3:.1f}".format(bL, wL, bR, wR))
    return (bL, wL), (bR, wR)

# --- Logging ---
log_dir = "/home/robot/logs"
try:
    os.makedirs(log_dir)
except OSError:
    pass
csv_path = "/home/robot/logs/line_simple_{0}_t2.csv".format(time.strftime("%Y%m%d_%H%M%S"))
f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","Lr","Rr","L","R","err","derr","base","cmdL","cmdR"])

# --- Parámetros del seguidor (PD) ---
BASE = 24            # velocidad base
KP   = 55.0
KD   = 8.0
TURN_CLAMP  = 35
SPEED_CLAMP = 70
DEADZONE    = 12.0

(bL,wL),(bR,wR) = calibrate_lr()
print("Pulsa para arrancar...")
wait_press_release(touch)
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

prev_err = 0.0
t0 = time.time()
t_prev = t0

try:
    while True:
        # Lecturas
        Lr = csL.reflected_light_intensity
        Rr = csR.reflected_light_intensity
        L = norm(Lr, bL, wL)  # 0=negro, 1=blanco
        R = norm(Rr, bR, wR)

        # Error: queremos la línea bajo L (negro) y blanco bajo R -> error positivo si me voy a la derecha
        # Usamos pesos de oscuridad (1-L, 1-R)
        w_left, w_right = (1.0 - L), (1.0 - R)
        # pos ~ (-1..+1). Si hay más negro a la izq, pos<0; a la der, pos>0
        denom = (w_left + w_right) if (w_left + w_right) > 1e-6 else 1.0
        pos = (-w_left + w_right) / denom
        err = -pos  # err>0 => debo girar a la izquierda

        # Tiempo
        t_now = time.time()
        DT = max(0.01, t_now - t_prev)
        t_prev = t_now

        derr = (err - prev_err) / DT
        prev_err = err

        # Control
        steer = KP*err + KD*derr
        if steer > TURN_CLAMP:  steer = TURN_CLAMP
        if steer < -TURN_CLAMP: steer = -TURN_CLAMP

        cmdL = BASE - steer/2.0
        cmdR = BASE + steer/2.0

        # Límites
        if cmdL > SPEED_CLAMP:  cmdL = SPEED_CLAMP
        if cmdL < -SPEED_CLAMP: cmdL = -SPEED_CLAMP
        if cmdR > SPEED_CLAMP:  cmdR = SPEED_CLAMP
        if cmdR < -SPEED_CLAMP: cmdR = -SPEED_CLAMP

        # Dead-zone para que siempre se muevan
        cmdL = apply_deadzone(cmdL, DEADZONE)
        cmdR = apply_deadzone(cmdR, DEADZONE)

        lm.on(SpeedPercent(cmdL))
        rm.on(SpeedPercent(cmdR))

        # Log
        writer.writerow([
            round(t_now - t0,3),
            int(Lr), int(Rr),
            round(L,3), round(R,3),
            round(err,3), round(derr,3),
            BASE, round(cmdL,1), round(cmdR,1)
        ])
        f.flush()

        # Parar si mantienes presionado >1.2s
        if touch.is_pressed:
            t_press = time.time()
            while touch.is_pressed: time.sleep(0.01)
            if time.time() - t_press > 1.2:
                break

        # ~25 Hz
        time.sleep(0.04)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    leds.all_off()
    f.close()
    print("CSV:", csv_path)
