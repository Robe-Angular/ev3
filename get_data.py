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
BASE_MAX = 20      # % en rectas
BASE_MIN = 12       # % en curvas duras
K_SPEED  = 16      # caída por |err| (suave = 18..28)

# Control PD (realmente PD; integral NO para evitar windup)
KP = 45.0          # 40..120
KD = 10.0          # 6..16
TURN_CLAMP   = 40  # tope giro
SPEED_CLAMP  = 55  # tope motor
MOTOR_GAIN   = 1.00
MAX_DELTA    = 8.0 # rampa real (ANTES 60 → demasiado alto)

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

def _avg_reading(sensor, samples=8, delay=0.01):
    s = 0.0
    for _ in range(samples):
        s += sensor.reflected_light_intensity
        time.sleep(delay)
    return s / samples

def calibrate():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    snd.speak('Place all sensors on WHITE, then press.')
    wait_press_release(touch)
    wL = _avg_reading(csL); wC = _avg_reading(csC); wR = _avg_reading(csR)

    snd.speak('Place all sensors on BLACK line, then press.')
    wait_press_release(touch)
    bL = _avg_reading(csL); bC = _avg_reading(csC); bR = _avg_reading(csR)

    # Autocorrección si quedaron invertidos (por luz/altura)
    if wL <= bL: wL, bL = bL, wL
    if wC <= bC: wC, bC = bC, wC
    if wR <= bR: wR, bR = bR, wR

    # Separación mínima de 5 puntos (evita hi==lo)
    MIN_DELTA = 5.0
    if abs(wL-bL) < MIN_DELTA: wL = bL + MIN_DELTA
    if abs(wC-bC) < MIN_DELTA: wC = bC + MIN_DELTA
    if abs(wR-bR) < MIN_DELTA: wR = bR + MIN_DELTA

    # --- Validación: línea bajo el sensor CENTRAL ---
    leds.set_color('LEFT','ORANGE'); leds.set_color('RIGHT','ORANGE')
    snd.speak('Center the BLACK line under CENTER sensor, then press.')
    print(">>> Coloca la LINEA bajo el sensor CENTRAL y pulsa el touch... <<<")
    wait_press_release(touch)
    midC = _avg_reading(csC)

    # Normaliza C: 0=negro(línea), 1=blanco(fondo)
    C_norm = (midC - bC) / max(1.0, (wC - bC))
    C_norm = max(0.0, min(1.0, C_norm))
    print(f"[VALID] C on LINE -> raw={midC:.1f} | norm={C_norm:.2f}")

    LINE_ON_CENTER_MAX = 0.25  # si tu línea es clarita, sube a 0.30–0.35
    if C_norm > LINE_ON_CENTER_MAX:
        snd.speak('Center validation failed. Recalibrate.')
        leds.set_color('LEFT','RED'); leds.set_color('RIGHT','RED')
        print("❌ El sensor central NO ve negro suficiente. Repite calibración (ajusta altura/luz).")
        return calibrate()

    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    snd.speak('Calibration OK.')
    print(f"✅ Calibración OK. L: {bL:.1f}/{wL:.1f}  C: {bC:.1f}/{wC:.1f}  R: {bR:.1f}/{wR:.1f}")
    return (bL,wL), (bC,wC), (bR,wR)


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

snd.speak('Comm-link online.')
calibL, calibC, calibR = calibrate()
snd.speak('Systems functional.')
wait_press_release(touch)
snd.speak('Go ahead, TACCOM.')
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

t_prev = time.time()
t0 = t_prev
prev_err = 0.0
d_filt = 0.0
last_seen_dir = 0
running = True
prev_cmdL = 0.0
prev_cmdR = 0.0

lm.on(SpeedPercent(15)); rm.on(SpeedPercent(15))
time.sleep(0.5)
lm.stop(); rm.stop()
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
            # if C < 0.35 and base > (BASE_MIN + 1):  # si centro ve negro → precaución
            #     base = BASE_MIN + 1
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
