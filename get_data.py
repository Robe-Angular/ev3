#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time, os, csv

# --------- Hardware ----------
lm = LargeMotor(OUTPUT_B)
rm = LargeMotor(OUTPUT_C)
lm.stop_action = 'brake'
rm.stop_action = 'brake'
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
snd = Sound(); leds = Leds()

# --------- Parámetros ----------
# Velocidad adaptativa: base cae cuando el error es grande
BASE_MAX = 12      # % en rectas
BASE_MIN = 6       # % en curvas duras
K_SPEED  = 24      # caída por |err| (suave = 18..28)

# Control PD (realmente PD; integral NO para evitar windup)
KP = 45.0          # 40..120
KD = 12.0          # 6..16
TURN_CLAMP   = 50  # tope giro
SPEED_CLAMP  = 55  # tope motor
MOTOR_GAIN   = 0.60
MAX_DELTA    = 6.0 # rampa real (ANTES 60 → demasiado alto)

# Señal de giro por si queda invertido (+1 o -1)
TURN_SIGN    = +1  # si gira al revés, pon -1

# Línea perdida / esquina usando NORMALIZADOS
# (0 ~ negro línea, 1 ~ blanco fondo tras calibración)
LINE_LOST_SUMW = 0.08      # si (wL+wC+wR)<esto ⇒ casi todo blanco
CORNER_DEEP    = 0.20      # “muy negro” ~ <0.20
CORNER_CLEAR   = 0.70      # “muy blanco” ~ >0.70
PIVOT_SPEED    = 10
SEARCH_SPEED   = 8

# Derivativo suavizado (filtro exponencial)
D_ALPHA        = 0.35

# --------- Utils ----------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wait_press_release(t):
    while not t.is_pressed: time.sleep(0.01)
    while t.is_pressed:     time.sleep(0.01)

# Calibración por sensor (blanco/negro)
def calibrate():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    snd.speak('Place on white and press.')
    wait_press_release(touch)
    wL, wC, wR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

    snd.speak('Place on black and press.')
    wait_press_release(touch)
    bL, bC, bR = csL.reflected_light_intensity, csC.reflected_light_intensity, csR.reflected_light_intensity

    # evita (hi==lo)
    if abs(wL-bL) < 5: wL = bL + 5
    if abs(wC-bC) < 5: wC = bC + 5
    if abs(wR-bR) < 5: wR = bR + 5
    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    return (bL,wL),(bC,wC),(bR,wR)

def norm(v, lo, hi):
    d = float(hi-lo); 
    if d < 1: d = 1.0
    x = (v - lo)/d
    if x < 0: x = 0.0
    if x > 1: x = 1.0
    return x

def slew(prev_cmd, target_cmd, max_delta):
    if target_cmd > prev_cmd + max_delta: return prev_cmd + max_delta
    if target_cmd < prev_cmd - max_delta: return prev_cmd - max_delta
    return target_cmd

# --------- Inicio / CSV ----------
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
os.makedirs(log_dir, exist_ok=True)
csv_path = os.path.join(log_dir, "ev3_log_" + str(ts) + ".csv")
f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","pos","err","derr","steer","base","cmdL","cmdR"])

snd.speak('Calibrating.')
calibL, calibC, calibR = calibrate()
snd.speak('Tap to start.')
wait_press_release(touch)
snd.speak('Go.')
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

t_prev = time.time()
t0 = t_prev
prev_err = 0.0
d_filt = 0.0
last_seen_dir = 0
running = True
prev_cmdL = 0.0
prev_cmdR = 0.0

try:
    while running:
        # Lecturas crudas
        Lr = csL.reflected_light_intensity
        Cr = csC.reflected_light_intensity
        Rr = csR.reflected_light_intensity

        # Normaliza 0..1 (0 negro, 1 blanco)
        L = norm(Lr, *calibL)
        C = norm(Cr, *calibC)
        R = norm(Rr, *calibR)

        # Pesos de "oscuridad" (línea)
        wL, wC, wR = 1.0-L, 1.0-C, 1.0-R
        sumw = wL + wC + wR

        # DT real
        t_now = time.time()
        DT = max(0.01, t_now - t_prev)   # evita DT=0
        t_prev = t_now

        # Estados especiales (con NORMALIZADOS)
        line_lost = (sumw < LINE_LOST_SUMW)
        corner_left  = (L <= CORNER_DEEP and C >= CORNER_CLEAR and R >= CORNER_CLEAR)
        corner_right = (R <= CORNER_DEEP and C >= CORNER_CLEAR and L >= CORNER_CLEAR)

        # Defaults para log
        pos = 0.0; err = 0.0; derr = 0.0; steer = 0.0; base = BASE_MIN
        cmdL_target = 0.0; cmdR_target = 0.0

        if corner_left:
            cmdL_target = -PIVOT_SPEED
            cmdR_target = +PIVOT_SPEED
            last_seen_dir = -1
        elif corner_right:
            cmdL_target = +PIVOT_SPEED
            cmdR_target = -PIVOT_SPEED
            last_seen_dir = +1
        elif line_lost:
            # búsqueda hacia el último lugar donde se "vio" más oscuro
            turn = SEARCH_SPEED
            if last_seen_dir <= 0:
                cmdL_target = -turn; cmdR_target = +turn
            else:
                cmdL_target = +turn; cmdR_target = -turn
            base = SEARCH_SPEED
        else:
            # Centroide: -1 (L), 0 (C), +1 (R)
            pos = (-1.0*wL + 1.0*wR) / (sumw if sumw>1e-6 else 1.0)
            # Error (0 deseado)
            err = -pos  # si gira al revés, cambia a err = +pos
            # Derivativo con filtro
            d_raw = (err - prev_err) / DT
            d_filt = (1.0 - D_ALPHA)*d_filt + D_ALPHA*d_raw
            derr = d_filt
            prev_err = err

            # Velocidad base adaptativa
            base = BASE_MAX - K_SPEED * abs(err)
            if C < 0.35 and base > (BASE_MIN + 1):  # si centro ve negro → precaución
                base = BASE_MIN + 1
            base = clamp(base, BASE_MIN, BASE_MAX)

            # PID (aquí PD) + posible inversión
            steer = TURN_SIGN * (KP*err + KD*derr)
            steer = clamp(steer, -TURN_CLAMP, TURN_CLAMP)

            # Diferencial
            cmdL_target = base - steer/2.0
            cmdR_target = base + steer/2.0

            # Recuerda hacia dónde había más "negro"
            last_seen_dir = -1 if (wL > wR and wL > wC) else (+1 if (wR > wL and wR > wC) else last_seen_dir)

        # Salida común
        cmdL_target = clamp(cmdL_target, -SPEED_CLAMP, SPEED_CLAMP)
        cmdR_target = clamp(cmdR_target, -SPEED_CLAMP, SPEED_CLAMP)

        cmdL = clamp(slew(prev_cmdL, cmdL_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)
        cmdR = clamp(slew(prev_cmdR, cmdR_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)
        prev_cmdL, prev_cmdR = cmdL, cmdR

        lm.on(SpeedPercent(cmdL * MOTOR_GAIN))
        rm.on(SpeedPercent(cmdR * MOTOR_GAIN))

        # Log
        writer.writerow([
            round(t_now - t0,3),
            int(Lr), int(Cr), int(Rr),
            round(pos,3), round(err,3), round(derr,3),
            round(steer,3),
            int(base),
            round(cmdL,1), round(cmdR,1)
        ])
        f.flush()

        # Touch: pausa/salida
        if touch.is_pressed:
            lm.stop(); rm.stop()
            t_press = time.time()
            wait_press_release(touch)
            if time.time() - t_press > 1.2:
                running = False
                break

        # target ~25 Hz
        time.sleep(max(0.0, 0.04 - (time.time()-t_now)))

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    leds.all_off()
    snd.speak('Acknowledged H.Q.')
    f.close()
    print("CSV:", csv_path)
