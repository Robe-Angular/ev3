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

# --- Blind steer más suave/proporcional ---
BLIND_SUMW        = 0.14   # antes 0.20 (activa blind solo si los lados están MUY blancos)
BLIND_C_TH        = 0.68   # sube el requisito de C claro
BLIND_STEER_MIN   = 8.0    # antes 16.0 → giro mínimo más chico
BLIND_STEER_MAX   = 14.0   # tope de giro en blind (antes no teníamos)
BLIND_BASE_DROP   = 4      # baja la base en blind para no “salirte patinando”
STEER_ZERO_TH     = 0.3    # antes 0.5 (deja pasar giros chicos útiles)

# SEARCH sin reversa
SEARCH_FWD_MIN  = 10   # si patina, 12
SEARCH_TURN     = 5    # era 6
SEARCH_BUMP_FWD = 14
SEARCH_TIMEOUT  = 0.60
CENTER_ON_LINE_TH = 0.55  # baja de 0.60 → no salga de SEARCH por gris tenue

# Línea perdida robusta
# Más tolerante para re-adquirir y más difícil caer a SEARCH
CLEAR_TH       = 0.56      # antes 0.68
LINE_LOST_DEB  = 0.22      # antes 0.10
REACQ_W        = 0.14      # antes 0.16
# --------- Parámetros ----------
# Velocidad adaptativa: base cae cuando el error es grande
BASE_MAX = 26      # % en rectas
BASE_MIN = 16       # % en curvas duras
K_SPEED  = 28      # caída por |err| (suave = 18..28)

# Dead-zone (nuevo)
DEADZONE_MIN = 12.0  # % mínimo efectivo de motor

# Control PD (realmente PD; integral NO para evitar windup)
KP = 42.0          # 40..120
KD = 8.0          # 6..16
TURN_CLAMP   = 30  # tope giro
SPEED_CLAMP  = 70  # tope motor
MOTOR_GAIN   = 1.00
MAX_DELTA    = 4.0 # rampa real (ANTES 60 → demasiado alto)

# Esquina aguda (dos sensores negros)
HARD_DEEP     = 0.30    # antes 0.26
HARD_CLEAR    = 0.70    # antes 0.62
HARD_MIN_HOLD = 0.40    # 0.40–0.45
HARD_ARC_FWD   = 14        # % avance en el arco duro
HARD_ARC_DIFF  = 20       # % diferencial para “morder” el giro duro

# Señal de giro por si queda invertido (+1 o -1)
TURN_SIGN    = +1  # si gira al revés, pon -1

# Línea perdida / esquina usando NORMALIZADOS
# (0 ~ negro línea, 1 ~ blanco fondo tras calibración)
LINE_LOST_SUMW = 0.08      # si (wL+wC+wR)<esto ⇒ casi todo blanco
PIVOT_SPEED    = 12
SEARCH_SPEED   = 11

# --- Esquinas robustas ---
C_HOOK        = 0.15    # 0.12–0.18; si el centro es más gris, súbelo
CORNER_DEEP   = 0.20    # antes 0.18
CORNER_CLEAR  = 0.82    # antes 0.78
CORNER_DEBOUNCE = 0.10  # antes 0.08


CORNER_MIN_HOLD  = 0.28   # s que nos quedamos en modo esquina como mínimo
CORNER_MAX_HOLD  = 0.80   # s de seguridad para no quedarnos atrapados
CORNER_PIVOT     = 11      # % base para pivot
CORNER_ARC_FWD   = 12      # % componente hacia adelante para escapar
CORNER_ARC_DIFF  = 16     # % diferencial para “morder” la esquina

# Derivativo suavizado (filtro exponencial)
D_ALPHA = 0.30
P_ALPHA = 0.25   # suaviza pos
S_ALPHA = 0.35   # suaviza mando de giro

# --- Nuevos (añade estos) ---
CORNER_FWD_MIN     = 12          # ambas ruedas ≥ este % en esquina
CORNER_COOLDOWN    = 0.18        # tras salir de esquina, ignora nueva esquina

line_lost_deb_timer = 0.0
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

def apply_deadzone(cmd, dz=DEADZONE_MIN):
    if cmd > 0 and cmd < dz:  return dz
    if cmd < 0 and cmd > -dz: return -dz
    return cmd

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

def apply_deadzone_search(cmd, dz=SEARCH_FWD_MIN):
    # En SEARCH nunca dejamos ir por debajo del mínimo hacia adelante
    return dz if cmd < dz else cmd

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
wL_prev = wC_prev = wR_prev = 0.0
d_wL = d_wC = d_wR = 0.0
warmup = 0.0
corner_cool = 0
search_timer = 0.0

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
        # --- detección hard-turn (dos sensores contiguos muy negros) ---
        hard_left  = (L <= HARD_DEEP and C <= HARD_DEEP and R >= HARD_CLEAR)
        hard_right = (R <= HARD_DEEP and C <= HARD_DEEP and L >= HARD_CLEAR)
        # DT real
        t_now = time.time()
        DT = max(0.01, t_now - t_prev)   # evita DT=0
        corner_cool = max(0.0, corner_cool - DT)
        warmup += DT
        KD_eff = KD if warmup >= 0.30 else 0.0
        t_prev = t_now
        search_timer = search_timer + DT if state == STATE_SEARCH else 0.0


        # Pesos de "oscuridad" (línea)
        wL, wC, wR = 1.0-L, 1.0-C, 1.0-R
        sumw = wL + wC + wR
        d_wL = (wL - wL_prev) / DT
        d_wR = (wR - wR_prev) / DT
        # (opcional) suaviza un poco
        DW_ALPHA = 0.4
        try:
            d_wL = (1.0 - DW_ALPHA)*d_wL_f + DW_ALPHA*d_wL
            d_wR = (1.0 - DW_ALPHA)*d_wR_f + DW_ALPHA*d_wR
        except NameError:
            pass
        d_wL_f, d_wR_f = d_wL, d_wR

        wL_prev, wR_prev = wL, wR

        # ----- detecciones básicas -----
        # Línea perdida robusta: los 3 ven muy blanco
        # Línea perdida robusta: los 3 ven muy blanco
        line_lost_raw = (L >= CLEAR_TH and C >= CLEAR_TH and R >= CLEAR_TH)

        # --- LATCH DE TENDENCIA ANTES DE ENTRAR A SEARCH ---
        # Mezcla nivel (wR-wL) y tendencia (d_wR-d_wL):
        #  - si R venía más oscuro/oscureciéndose → +1 (buscar a la derecha)
        #  - si L venía más oscuro/oscureciéndose → -1 (buscar a la izquierda)
        if line_lost_raw and state == STATE_NORMAL:
            trend_score = 0.6 * (wR - wL) + 0.4 * (d_wR - d_wL)
            # márgenes pequeños para no fl apar por ruido
            if abs(wR - wL) > 0.04 or abs(d_wR - d_wL) > 0.04:
                last_seen_dir = +1 if trend_score > 0 else -1
        # -----------------------------------------------

        # debounce para evitar parpadeos
        if line_lost_raw:
            line_lost_deb_timer += DT
        else:
            line_lost_deb_timer = 0.0

        line_lost = (line_lost_deb_timer >= LINE_LOST_DEB)

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
            
            # ---------- NUEVO BLOQUE DE COOLDOWN ----------
            if corner_cool > 0.0:
                # En enfriamiento: NO detectes esquina nueva
                is_corner = False
                is_corner_raw = False
                hard_left = hard_right = False
                skip_corner = True
            else:
                # Detectores normales (solo si NO hay cooldown)
                hard_left  = (L <= HARD_DEEP and C <= HARD_DEEP and R >= HARD_CLEAR)
                hard_right = (R <= HARD_DEEP and C <= HARD_DEEP and L >= HARD_CLEAR)
                # 'is_corner' / 'is_corner_raw' ya los traes calculados arriba con debounce
                skip_corner = False
            # ---------- FIN COOLDOWN ----------

            if (not skip_corner) and (C <= C_HOOK) and (L >= CLEAR_TH) and (R >= CLEAR_TH):
                # Hook central: decide lado por tendencia reciente
                side_by_trend = +1 if (d_wR > d_wL) else -1
                corner_side = side_by_trend if (d_wL != 0.0 or d_wR != 0.0) else (last_seen_dir if last_seen_dir != 0 else +1)

                last_seen_dir = corner_side
                state = STATE_CORNER
                corner_hold = 0.0
                corner_is_hard = False   # esquina normal
            elif (not skip_corner) and (hard_left or hard_right):
                corner_side = -1 if hard_left else +1
                last_seen_dir = corner_side
                state = STATE_CORNER
                corner_hold = 0.0
                corner_is_hard = True    # hard-turn
            elif (not skip_corner) and is_corner:
                corner_side = +1 if (wR > wL) else -1
                last_seen_dir = corner_side
                state = STATE_CORNER
                corner_hold = 0.0
            elif line_lost and (wL + wC + wR) <= LINE_LOST_SUMW:
                state = STATE_SEARCH
            # ---------- CONTROL PD SOLO SI SIGUES EN NORMAL ----------
        if state == STATE_NORMAL:
            # PD normal
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

            # Si metiste el “warmup” de KD, usa KD_eff; si no, deja KD:
            KD_eff = KD if warmup >= 0.30 else 0.0  # opcional
            steer = TURN_SIGN * (KP*err + KD_eff*derr)
            steer = clamp(steer, -TURN_CLAMP, TURN_CLAMP)

            try:
                steer = (1.0 - S_ALPHA)*steer_prev + S_ALPHA*steer
            except NameError:
                pass
            steer_prev = steer

            if abs(steer) < 1.5:
                steer = 0.0

            # --- Blind steer proporcional SUAVE ---
            # Casi no hay línea en los lados y el centro está claro → guía suave hacia last_seen_dir
            if (wL + wR) < BLIND_SUMW and C >= BLIND_C_TH:
                side = last_seen_dir if last_seen_dir != 0 else +1
                # cuánto más oscuro está un lado que el otro (0..1 aprox)
                g = abs(wR - wL)
                # escalar 8..14 aprox según g (muy pequeño si la diferencia es mínima)
                steer_mag = BLIND_STEER_MIN + 20.0 * g
                if steer_mag > BLIND_STEER_MAX:
                    steer_mag = BLIND_STEER_MAX

                steer = side * steer_mag
                base  = clamp(base - BLIND_BASE_DROP, BASE_MIN, BASE_MAX)

            # anulación de giros mínimos (más permisiva)
            if abs(steer) < STEER_ZERO_TH:
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
            if C <= 0.15:
                base = max(5, base - 2)  # quita un pelín de avance para morder mejor


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
            # --- NUEVO: obliga avance mínimo en ambas ruedas ---
            cmdL_target = max(cmdL_target, CORNER_FWD_MIN)
            cmdR_target = max(cmdR_target, CORNER_FWD_MIN)

            pos = side * 1.0; err = pos; derr = 0.0; steer = diff  # solo para log

            # criterios de salida: mínimo tiempo + ya no corner o centro toca línea o timeout
            center_on_line = (C <= 0.50)       # ajusta si tu línea es gris
            not_corner_now = (not is_corner_raw) and (not hard_left) and (not hard_right)
             # --- ESTA ES LA LÍNEA CLAVE QUE PREGUNTAS ---
            if (corner_hold >= min_hold and not_corner_now) or center_on_line or (corner_hold >= CORNER_MAX_HOLD):
                state = STATE_NORMAL
                corner_deb = 0.0   # evita reenganchar de inmediato
                corner_hold = 0.0  # resetea el latch
                # (opcional) recuerda el último lado visto:
                last_seen_dir = side
                corner_is_hard = False

        elif state == STATE_SEARCH:
            side = last_seen_dir if last_seen_dir != 0 else +1
            base = 0  # no usamos 'base' aquí

            # Giro suave + avance (sin reversa)
            fwd  = SEARCH_FWD_MIN
            turn = SEARCH_TURN

            # Si lleva mucho sin re-adquirir, cambia estrategia: más avance y cambia el lado
            if search_timer >= SEARCH_TIMEOUT:
                side = -side              # invierte espiral
                last_seen_dir = side
                fwd  = SEARCH_BUMP_FWD    # empujón hacia delante
                turn = SEARCH_TURN - 2    # un poco menos de giro para proyectar
                search_timer = 0.0        # reinicia timer

            # Comandos crudos
            cmdL_target = fwd - side * turn
            cmdR_target = fwd + side * turn

            # --- FORZAR AVANCE: nunca reversa en SEARCH ---
            if cmdL_target < SEARCH_FWD_MIN: cmdL_target = SEARCH_FWD_MIN
            if cmdR_target < SEARCH_FWD_MIN: cmdR_target = SEARCH_FWD_MIN

            pos = 0.0; err = 0.0; derr = 0.0; steer = side*turn  # log

            # Salida de SEARCH si vemos “algo” o esquina
            reacq = (wL >= REACQ_W) or (wC >= REACQ_W) or (wR >= REACQ_W) or (C <= CENTER_ON_LINE_TH)
            if reacq or is_corner:
                state = STATE_NORMAL
                corner_deb = 0.0
                line_lost_deb_timer = 0.0
                search_timer = 0.0


        # Salida común
        cmdL_target = clamp(cmdL_target, -SPEED_CLAMP, SPEED_CLAMP)
        cmdR_target = clamp(cmdR_target, -SPEED_CLAMP, SPEED_CLAMP)

        cmdL = clamp(slew(prev_cmdL, cmdL_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)
        cmdR = clamp(slew(prev_cmdR, cmdR_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)

        prev_cmdL, prev_cmdR = cmdL, cmdR

        # Dead-zone según estado
        if state == STATE_SEARCH:
            cmdL = apply_deadzone_search(cmdL, SEARCH_FWD_MIN)
            cmdR = apply_deadzone_search(cmdR, SEARCH_FWD_MIN)
        else:
            cmdL = apply_deadzone(cmdL)
            cmdR = apply_deadzone(cmdR)

        lm.on(SpeedPercent(cmdL * MOTOR_GAIN))
        rm.on(SpeedPercent(cmdR * MOTOR_GAIN))


        # Log
        writer.writerow([
            round(t_now - t0,3),
            int(Lr), int(Cr), int(Rr),
            round(pos,3), round(err,3), round(derr,3),
            round(steer,3),
            int(base),
            round(cmdL,1), round(cmdR,1),
            state
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
    snd.speak("Ackonwledged H.Q.")
    f.close()
    print("CSV:", csv_path)
