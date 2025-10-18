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
BASE_MAX = 16      # % en rectas
BASE_MIN = 10       # % en curvas duras
K_SPEED  = 22      # caída por |err| (suave = 18..28)

# Control PD (realmente PD; integral NO para evitar windup)
KP = 32.0          # 40..120
KD = 14.0          # 6..16
TURN_CLAMP   = 30  # tope giro
SPEED_CLAMP  = 55  # tope motor
MOTOR_GAIN   = 1.00
MAX_DELTA    = 6.0 # rampa real (ANTES 60 → demasiado alto)

# Esquina aguda (dos sensores negros)
HARD_DEEP   = 0.28   # 0..1 (negro). Sube a 0.32 si tu negro es muy oscuro
HARD_CLEAR  = 0.65   # 0..1 (blanco). Baja a 0.60 si el fondo es gris
HARD_MIN_HOLD = 0.28 # s de compromiso mínimo (más que corner normal)
HARD_ARC_FWD  = 8    # % avance en el arco duro
HARD_ARC_DIFF = 14   # % diferencial para morder el giro duro

# Señal de giro por si queda invertido (+1 o -1)
TURN_SIGN    = +1  # si gira al revés, pon -1

# Línea perdida / esquina usando NORMALIZADOS
# (0 ~ negro línea, 1 ~ blanco fondo tras calibración)
LINE_LOST_SUMW = 0.08      # si (wL+wC+wR)<esto ⇒ casi todo blanco
PIVOT_SPEED    = 10
SEARCH_SPEED   = 8

# --- Esquinas robustas ---
CORNER_DEEP      = 0.22   # lado muy negro (0..1 normalizado)
CORNER_CLEAR     = 0.75   # los otros muy blancos
CORNER_DEBOUNCE  = 0.12   # s que debe durar la condición para aceptar esquina
CORNER_MIN_HOLD  = 0.18   # s que nos quedamos en modo esquina como mínimo
CORNER_MAX_HOLD  = 0.60   # s de seguridad para no quedarnos atrapados
CORNER_PIVOT     = 9      # % base para pivot
CORNER_ARC_FWD   = 7      # % componente hacia adelante para escapar
CORNER_ARC_DIFF  = 12     # % diferencial para “morder” la esquina

# Derivativo suavizado (filtro exponencial)
D_ALPHA        = 0.35
P_ALPHA = 0.30   # suaviza pos
S_ALPHA = 0.40   # suaviza mando de giro


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
    print("[VALID] C on LINE -> raw={:.1f} | norm={:.2f}".format(midC, C_norm))

    LINE_ON_CENTER_MAX = 0.25  # si tu línea es clarita, sube a 0.30–0.35
    if C_norm > LINE_ON_CENTER_MAX:
        snd.speak('Center validation failed. Recalibrate.')
        leds.set_color('LEFT','RED'); leds.set_color('RIGHT','RED')
        print("El sensor central NO ve negro suficiente. Repite calibración (ajusta altura/luz).")
        return calibrate()

    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    snd.speak('Calibration OK.')
    print("Calibracion OK. L: {0:.1f}/{1:.1f}  C: {2:.1f}/{3:.1f}  R: {4:.1f}/{5:.1f}".format(bL, wL, bC, wC, bR, wR))
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


STATE_NORMAL = 0
STATE_CORNER = 1
STATE_SEARCH = 2

state = STATE_NORMAL
corner_deb = 0.0    # acumula tiempo de condición de esquina
corner_hold = 0.0   # tiempo dentro del estado esquina (latch)
corner_side = 0     # -1 izquierda, +1 derecha

# header
writer.writerow(["t","L","C","R","pos","err","derr","steer","base","cmdL","cmdR","state"])
# fila
writer.writerow([... , state])

snd.speak('Comm-link online.')
calibL, calibC, calibR = calibrate()

# --- TEST RÁPIDO: línea bajo el sensor IZQUIERDO (1s, sin mover el robot) ---
print("TEST IZQ: coloca la LiNEA bajo el sensor IZQUIERDO (L) durante 1s...")
t_test = time.time()
while time.time() - t_test < 1.0:
    Lr = csL.reflected_light_intensity
    Rr = csR.reflected_light_intensity
    print("Lr=", Lr, "  Rr=", Rr)
    time.sleep(0.1)

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

corner_is_hard = False

try:
    while running:

        # --- detección hard-turn (dos sensores contiguos muy negros) ---
        hard_left  = (L <= HARD_DEEP and C <= HARD_DEEP and R >= HARD_CLEAR)
        hard_right = (R <= HARD_DEEP and C <= HARD_DEEP and L >= HARD_CLEAR)

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

        # ----- detecciones básicas -----
        line_lost = (sumw < LINE_LOST_SUMW)

        # “esquina cruda”: exactamente 1 muy negro y alguno muy blanco
        darkL = (L <= CORNER_DEEP); darkC = (C <= CORNER_DEEP); darkR = (R <= CORNER_DEEP)
        clearL = (L >= CORNER_CLEAR); clearC = (C >= CORNER_CLEAR); clearR = (R >= CORNER_CLEAR)
        is_corner_raw = ((darkL + darkC + darkR) == 1) and (clearL or clearC or clearR)

        # debounce de esquina (no uses latch aquí)
        if is_corner_raw:
            corner_deb += DT
        else:
            corner_deb = 0.0

        is_corner = (corner_deb >= CORNER_DEBOUNCE)

        # ----- FSM -----
        if state == STATE_NORMAL:
            # por defecto (por si no hay rama), inicializa salidas “neutras”
            pos = 0.0; err = 0.0; derr = 0.0; steer = 0.0; base = BASE_MIN
            cmdL_target = 0.0; cmdR_target = 0.0
            # PRIORIDAD: hard-turn
            if hard_left or hard_right:
                corner_side = -1 if hard_left else +1
                last_seen_dir = corner_side
                state = STATE_CORNER
                corner_hold = 0.0
                # etiqueta este corner como "duro" guardando params en locals
                corner_is_hard = True
            elif is_corner:
                corner_side = +1 if (wR > wL) else -1
                last_seen_dir = corner_side          # <--- añade esto
                state = STATE_CORNER
                corner_hold = 0.0
            elif line_lost:
                state = STATE_SEARCH
            else:
                # ---------- PD normal ----------
                pos_raw = (-1.0*wL + 1.0*wR) / (sumw if sumw>1e-6 else 1.0)

                
                try:
                    pos = (1.0 - P_ALPHA)*pos + P_ALPHA*pos_raw
                except NameError:
                    pos = pos_raw

                dpos = (pos - locals().get('pos_prev', pos)) / DT
                pos_prev = pos
                err  = -pos
                derr = -dpos   # derivada sobre medición → amortigua mejor

                base = clamp(BASE_MAX - K_SPEED*abs(err), BASE_MIN, BASE_MAX)

                steer = TURN_SIGN * (KP*err + KD*derr)
                steer = clamp(steer, -TURN_CLAMP, TURN_CLAMP)

                try:
                    steer = (1.0 - S_ALPHA)*steer_prev + S_ALPHA*steer
                except NameError:
                    pass
                steer_prev = steer

                if abs(steer) < 1.5:
                    steer = 0.0

                cmdL_target = base - steer/2.0
                cmdR_target = base + steer/2.0

                # recordar lado más oscuro por si toca SEARCH
                if (wL > wR and wL > wC): last_seen_dir = -1
                elif (wR > wL and wR > wC): last_seen_dir = +1

        elif state == STATE_CORNER:
            corner_hold += DT
            # arco de escape: un poco adelante + diferencial fuerte hacia el lado
            side = corner_side  # -1 izq, +1 der


            if corner_is_hard:
                fwd  = HARD_ARC_FWD
                diff = HARD_ARC_DIFF * side
                min_hold = HARD_MIN_HOLD
            else:
                fwd  = CORNER_ARC_FWD
                diff = CORNER_ARC_DIFF * side
                min_hold = CORNER_MIN_HOLD

            base = fwd
            cmdL_target = fwd + diff/2.0
            cmdR_target = fwd - diff/2.0
            pos = side * 1.0; err = -pos; derr = 0.0; steer = diff  # solo para log

            # criterios de salida: mínimo tiempo + ya no corner o centro toca línea o timeout
            center_on_line = (C <= 0.45)       # ajusta si tu línea es gris
            not_corner_now = (not is_corner_raw) and (not hard_left) and (not hard_right)
             # --- ESTA ES LA LÍNEA CLAVE QUE PREGUNTAS ---
            if (corner_hold >= CORNER_MIN_HOLD and not_corner_now) or center_on_line or (corner_hold >= CORNER_MAX_HOLD):
                state = STATE_NORMAL
                corner_deb = 0.0   # evita reenganchar de inmediato
                corner_hold = 0.0  # resetea el latch
                # (opcional) recuerda el último lado visto:
                last_seen_dir = side
                corner_is_hard = False

        elif state == STATE_SEARCH:
            side = last_seen_dir if last_seen_dir != 0 else +1
            turn = SEARCH_SPEED
            base = SEARCH_SPEED
            cmdL_target = -side * turn
            cmdR_target =  side * turn
            pos = 0.0; err = 0.0; derr = 0.0; steer = side*turn  # log

            # sal de búsqueda si vemos línea o esquina
            if sumw >= LINE_LOST_SUMW or is_corner:
                state = STATE_NORMAL
                corner_deb = 0.0


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
