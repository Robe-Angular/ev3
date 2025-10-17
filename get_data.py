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
# Configura el modo de paro de los motores (tu versión lo lee como propiedad)
lm.stop_action = 'brake'
rm.stop_action = 'brake'
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
snd = Sound(); leds = Leds()

# --------- Parámetros ----------
# Velocidad adaptativa: base cae cuando el error es grande
BASE_MAX = 12     # % máx en rectas
BASE_MIN = 6     # % mínimo en curvas duras
K_SPEED  = 28     # cuánto bajar por |error| (prueba 8..18)

# Control PD sobre posición (centroide): error en [-1,1]
KP = 45.0         # 60..140 (ajusta fino)
KD = 14.0          # 4..16   (amortigua quiebres)
DT = 0.04         # 20 ms (50 Hz)

# Saturación del giro y de los motores
TURN_CLAMP   = 50          # tope de mando diferencial
SPEED_CLAMP  = 55          # tope absoluto de % a motor

# Umbrales para perder/encontrar línea (valores 0..100 de reflectancia)
# Si usas pista negra sobre blanco: "negro" ≈ 5..20, "blanco" ≈ 60..90
BLACK_MAX = 35     # <= negro
WHITE_MIN = 55     # >= blanco

# Detección de esquinas: patrón "uno muy negro, los otros muy blancos"
CORNER_DIFF = 15   # contraste mínimo para considerar giro agresivo
PIVOT_SPEED = 10   # % velocidad al pivotear en esquina
SEARCH_SPEED = 8  # % velocidad en búsqueda (cuando se pierde la línea)

# Escala global de mando (freno extra)
MOTOR_GAIN = 0.60   # 60% de lo calculado

# --------- Utils ----------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def wait_press_release(t):
    while not t.is_pressed:
        time.sleep(0.01)
    while t.is_pressed:
        time.sleep(0.01)

# Normaliza 0..1 usando blanco/negro fijos simples (rápido y práctico)
# n=0 ~ negro (línea), n=1 ~ blanco (fondo)
def normalize(v):
    lo = 10.0     # estima negro típico; afina si quieres
    hi = 90.0     # estima blanco típico; afina si quieres
    n = (v - lo) / max(1.0, (hi - lo))
    if n < 0: n = 0.0
    if n > 1: n = 1.0
    return n

# Rampa: limita el cambio máximo por ciclo (suaviza el mando)
def slew(prev_cmd, target_cmd, max_delta):
    if target_cmd > prev_cmd + max_delta:
        return prev_cmd + max_delta
    if target_cmd < prev_cmd - max_delta:
        return prev_cmd - max_delta
    return target_cmd

# --------- Inicio / CSV ----------
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
csv_path = os.path.join(log_dir, "ev3_log_" + str(ts) + ".csv")
f = open(csv_path, "w", newline="")
writer = csv.writer(f)
writer.writerow(["t","L","C","R","pos","err","derr","base","cmdL","cmdR"])

snd.speak('Comm-link online.')
leds.set_color('LEFT','RED'); leds.set_color('RIGHT','RED')
wait_press_release(touch)
snd.speak('Go ahead, Taccom.')
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

t0 = time.time()
prev_err = 0.0
last_seen_dir = 0          # -1 izquierda, +1 derecha (para búsqueda)
running = True
prev_cmdL = 0.0
prev_cmdR = 0.0
MAX_DELTA = 60.0   # % máximo de cambio por ciclo

try:
    
    while running:
        # stop con botón BACK del brick (triángulo)
        # ev3dev2 no tiene Button() simple aquí; usa ctrl+c si no
        # (si quieres BACK, dímelo y lo integro con ev3dev2.button)

        # Lee reflectancias crudas 0..100
        Lr = csL.reflected_light_intensity
        Cr = csC.reflected_light_intensity
        Rr = csR.reflected_light_intensity

        # Normaliza a 0..1 (0 negro línea, 1 blanco)
        L = normalize(Lr); C = normalize(Cr); R = normalize(Rr)

        # Peso "línea" como 1-n (más peso si más oscuro)
        wL = 1.0 - L; wC = 1.0 - C; wR = 1.0 - R
        sumw = wL + wC + wR

        # Detección de “línea perdida”
        line_lost = (sumw < 0.10)  # casi todo blanco
        # Detección de esquina (una muy negra y las otras claras)
        corner_left  = (Lr <= BLACK_MAX and Cr >= WHITE_MIN and Rr >= WHITE_MIN and (Cr - Lr) >= CORNER_DIFF)
        clamped = Cr - Rr
        corner_right = (Rr <= BLACK_MAX and Cr >= WHITE_MIN and Lr >= WHITE_MIN and clamped >= CORNER_DIFF)

        # ---- antes del if grande ----
        cmdL_target = 0.0
        cmdR_target = 0.0

        if corner_left:
            # Pivot agresivo hacia la izquierda
            cmdL_target = -PIVOT_SPEED
            cmdR_target = +PIVOT_SPEED
            last_seen_dir = -1
            pos = -1.0; err = 1.0  # log aproximado
            derr = 0.0
            base = PIVOT_SPEED
        elif corner_right:
            # Pivot agresivo hacia la derecha
            cmdL_target = +PIVOT_SPEED
            cmdR_target = -PIVOT_SPEED
            last_seen_dir = +1
            pos = +1.0; err = -1.0
            derr = 0.0
            base = PIVOT_SPEED
        elif line_lost:
            # Búsqueda: gira suave hacia el último lado visto
            turn = SEARCH_SPEED
            if last_seen_dir <= 0:
                cmdL_target = -turn
                cmdR_target = +turn
            else:
                cmdL_target = +turn
                cmdR_target = -turn
            pos = 0.0; err = 0.0; derr = 0.0; base = SEARCH_SPEED
        else:
            # Centroide en posiciones -1 (L), 0 (C), +1 (R)
            # pos en [-1,1], 0 = centrado en sensor central
            pos = (-1.0*wL + 0.0*wC + 1.0*wR) / (sumw if sumw>1e-6 else 1.0)

            # Error = deseado(0) - pos → tomamos err = -pos para girar hacia la línea
            err = -pos
            derr = (err - prev_err) / DT
            prev_err = err

            # Velocidad base cae con |error| (más curva = más lento)
            base = BASE_MAX - K_SPEED * abs(err)
            if C < 0.35 and base > (BASE_MIN + 1):
                base = BASE_MIN + 1
            if base < BASE_MIN:
                base = BASE_MIN
            if base > BASE_MAX:
                base = BASE_MAX

            # Mando diferencial PD
            turn = KP * err + KD * derr
            if turn < -TURN_CLAMP:
                turn = -TURN_CLAMP
            if turn > TURN_CLAMP:
                turn = TURN_CLAMP

            # Cálculo de velocidades ideales
            cmdL_target = base - turn / 2.0
            cmdR_target = base + turn / 2.0

            # Memoriza hacia dónde “vi” más peso por si se pierde
            last_seen_dir = -1 if (wL > wR and wL > wC) else (+1 if (wR > wL and wR > wC) else last_seen_dir)

        # ---- bloque común de salida (siempre) ----
        cmdL_target = clamp(cmdL_target, -SPEED_CLAMP, SPEED_CLAMP)
        cmdR_target = clamp(cmdR_target, -SPEED_CLAMP, SPEED_CLAMP)

        cmdL = clamp(slew(prev_cmdL, cmdL_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)
        cmdR = clamp(slew(prev_cmdR, cmdR_target, MAX_DELTA), -SPEED_CLAMP, SPEED_CLAMP)
        prev_cmdL, prev_cmdR = cmdL, cmdR

        lm.on(SpeedPercent(cmdL * MOTOR_GAIN))
        rm.on(SpeedPercent(cmdR * MOTOR_GAIN))

        # Log
        writer.writerow([
            round(time.time()-t0,3),
            int(Lr), int(Cr), int(Rr),
            round(pos,3), round(err,3), round(derr,3),
            int(base),
            int(lm.speed_sp) if hasattr(lm,'speed_sp') else 0,
            int(rm.speed_sp) if hasattr(rm,'speed_sp') else 0
        ])
        f.flush()

        # Start/Stop con Touch (pausa/salida rápida)
        if touch.is_pressed:
            lm.on(SpeedPercent(0)); rm.on(SpeedPercent(0))
            lm.stop(); rm.stop()
            snd.beep()
            # Espera otra pulsación para continuar o mantenlo presionado >1.2s para salir
            t_press = time.time()
            wait_press_release(touch)
            if time.time() - t_press > 1.2:
                running = False
                break
            snd.beep()

        time.sleep(DT)

except KeyboardInterrupt:
    pass
finally:
    lm.on(SpeedPercent(0)); rm.on(SpeedPercent(0))
    lm.stop(); rm.stop()
    leds.all_off()
    snd.speak('Acknowledged H.Q.')
    print("CSV:", csv_path)
