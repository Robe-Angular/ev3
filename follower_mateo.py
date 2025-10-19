#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smart line follower (no reverse) with calibration, hysteresis, and arc search.
Python 3.5 compatible (EV3).
Ports:
  Motors:  B=LEFT (lm), C=RIGHT (rm)
  Colors:  1=L, 2=C, 3=R
  Touch:   4
CSV: /home/robot/logs/smart_log_YYYYmmdd_HHMMSS.csv
"""

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor, TouchSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time, os, csv

# ---------------- Hardware ----------------
lm = LargeMotor(OUTPUT_B)   # LEFT
rm = LargeMotor(OUTPUT_C)   # RIGHT
csL = ColorSensor(INPUT_1); csL.mode = 'COL-REFLECT'
csC = ColorSensor(INPUT_2); csC.mode = 'COL-REFLECT'
csR = ColorSensor(INPUT_3); csR.mode = 'COL-REFLECT'
touch = TouchSensor(INPUT_4)
snd = Sound(); leds = Leds()

# ---------------- CSV ----------------
ts = time.strftime("%Y%m%d_%H%M%S")
log_dir = "/home/robot/logs"; 
if not os.path.exists(log_dir): os.makedirs(log_dir)
csv_path = os.path.join(log_dir, "smart_log_" + ts + ".csv")
f = open(csv_path, "w")
writer = csv.writer(f)
writer.writerow(["t","Lr","Cr","Rr","L","C","R","wL","wC","wR","pos","base","steer","cmdL","cmdR","state"])
t0 = time.time()

# ---------------- Utils ----------------
def wait_press():
    while not touch.is_pressed: time.sleep(0.05)
    while touch.is_pressed:     time.sleep(0.05)

def avg(sensor, n=10, d=0.01):
    s = 0.0
    for _ in range(n):
        s += sensor.reflected_light_intensity
        time.sleep(d)
    return s/float(n)

# --- Ramp/Clamp ---
def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

MAX_DELTA = 6.0   # puntos % por ciclo (4..8)

_prevL = 0.0
_prevR = 0.0
def _slew(prev_cmd, target_cmd, max_delta=MAX_DELTA):
    if target_cmd > prev_cmd + max_delta: return prev_cmd + max_delta
    if target_cmd < prev_cmd - max_delta: return prev_cmd - max_delta
    return target_cmd


def norm(v, lo, hi):
    d = float(hi - lo)
    if d < 1.0: d = 1.0
    x = (v - lo) / d
    if x < 0.0: x = 0.0
    if x > 1.0: x = 1.0
    return x

# ---------------- Calibration ----------------
def calibrate():
    leds.set_color('LEFT','YELLOW'); leds.set_color('RIGHT','YELLOW')
    snd.speak("Place sensors on white and press the button.")
    wait_press()
    wL = avg(csL); wC = avg(csC); wR = avg(csR)
    print("White:", wL, wC, wR)

    snd.speak("Now place sensors on black line and press the button.")
    wait_press()
    bL = avg(csL); bC = avg(csC); bR = avg(csR)
    print("Black:", bL, bC, bR)

    # Ensure white > black per sensor
    if wL <= bL: wL, bL = bL, wL
    if wC <= bC: wC, bC = bC, wC
    if wR <= bR: wR, bR = bR, wR

    # thresholds by global average (hysteresis around WHITE)
    w_avg = (wL + wC + wR)/3.0
    b_avg = (bL + bC + bR)/3.0
    rng   = max(5.0, w_avg - b_avg)

    CLEAR_HI = b_avg + 0.70*rng   # “todo blanco” (entrar)
    CLEAR_LO = b_avg + 0.60*rng   # “sale de blanco” (salir)
    CENTER_ON_LINE = b_avg + 0.35*rng

    snd.speak("Calibration complete.")
    leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')
    print("Hyst -> CLEAR_HI={:.1f} CLEAR_LO={:.1f} CENTER_ON_LINE={:.1f}".format(
        CLEAR_HI, CLEAR_LO, CENTER_ON_LINE))
    return (bL,wL), (bC,wC), (bR,wR), CLEAR_HI, CLEAR_LO, CENTER_ON_LINE

calibL, calibC, calibR, CLEAR_HI, CLEAR_LO, CENTER_ON_LINE = calibrate()

# ---------------- Params (smart/simple) ----------------
# P follower
# ---------------- Params (smart/simple) ----------------
TURN_SIGN  = +1   # si gira al revés, pon -1
KP         = 48.0
TURN_CLAMP = 38.0
BASE_MAX   = 38.0
BASE_MIN   = 16.0
K_SPEED    = 22.0

# Blind steer (cuando centro claro y lados muy blancos)
BLIND_SUMW  = 0.14
BLIND_C_TH  = 0.65
BLIND_STEER = 10.0
BLIND_DROP  = 5.0

# Search (sin reversa)
SEARCH_FWD  = 18.0
SEARCH_TURN = 12.0
SEARCH_SIDE_SWITCH = 0.9


# Debounce / cooldown
LINE_LOST_DEB = 0.20   # s en “todo blanco” para declarar pérdida
REACQ_COOLDOWN = 0.20  # s ignorando “todo blanco” tras re-adquirir

# ---------------- State ----------------
FOLLOW, SEARCH = 0, 1
state = FOLLOW
last_side = +1   # +1=right, -1=left
lost_t = 0.0
cooldown_t = 0.0
search_t0 = 0.0

def motors_from_steer(base, steer):
    # Para girar a la DERECHA: left > right
    cmdL = base + steer/2.0
    cmdR = base - steer/2.0
    return cmdL, cmdR

def set_motors(cmdL, cmdR):
    # cmd en -100..100
    global _prevL, _prevR
    cmdL = clamp(cmdL, -100, 100)
    cmdR = clamp(cmdR, -100, 100)
    cmdL = _slew(_prevL, cmdL); _prevL = cmdL
    cmdR = _slew(_prevR, cmdR); _prevR = cmdR
    lm.on(SpeedPercent(int(cmdL)))
    rm.on(SpeedPercent(int(cmdR)))

# ---------------- Main loop ----------------
snd.speak("Ready. Press button to start.")
wait_press()
snd.speak("Line following started.")
leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

t_prev = time.time()

try:
    while True:
        if touch.is_pressed:
            snd.speak("Stopping.")
            break

        # Time step & timers
        t_now = time.time()
        DT = max(0.01, t_now - t_prev)
        t_prev = t_now
        if cooldown_t > 0.0: cooldown_t = max(0.0, cooldown_t - DT)

        # Read & normalize (0 black .. 1 white)
        Lr = csL.reflected_light_intensity
        Cr = csC.reflected_light_intensity
        Rr = csR.reflected_light_intensity
        L = norm(Lr, *calibL); C = norm(Cr, *calibC); R = norm(Rr, *calibR)

        # Darkness weights (line presence)
        wL, wC, wR = 1.0-L, 1.0-C, 1.0-R
        sumw = wL + wC + wR

        # Hysteresis for “all white”
        all_hi = (Lr >= CLEAR_HI and Cr >= CLEAR_HI and Rr >= CLEAR_HI)
        any_lo = (Lr <= CLEAR_LO or Cr <= CLEAR_LO or Rr <= CLEAR_LO)
        if all_hi and cooldown_t <= 0.0:
            lost_t += DT
        elif any_lo:
            lost_t = 0.0
        line_lost = (lost_t >= LINE_LOST_DEB)

        if state == FOLLOW:
            # Position from sides (center excluded for smoother curves)
            pos = (-1.0*wL + 1.0*wR) / (sumw if sumw > 1e-6 else 1.0)

            # Base speed adapts with error magnitude
            base = clamp(BASE_MAX - K_SPEED*abs(pos), BASE_MIN, BASE_MAX)

            # Proportional steer
            steer = clamp(TURN_SIGN * KP * pos, -TURN_CLAMP, TURN_CLAMP)

            # Blind steer: center very clear and sides very clear
            if (wL + wR) < BLIND_SUMW and C >= BLIND_C_TH:
                side = last_side if last_side != 0 else +1
                steer = steer + side * BLIND_STEER
                base  = max(BASE_MIN, base - BLIND_DROP)

            # Update last_side by darker side
            if wL > wR and wL > wC: last_side = -1
            elif wR > wL and wR > wC: last_side = +1

            # Go motors
            cmdL, cmdR = motors_from_steer(base, steer)
            set_motors(cmdL, cmdR)

            # Fall to SEARCH only if really lost
            if line_lost:
                state = SEARCH
                search_t0 = t_now
                snd.speak("Line lost. Search.")
                leds.set_color('LEFT','ORANGE'); leds.set_color('RIGHT','ORANGE')

            state_name = "FOLLOW"

        else:  # SEARCH
            side = last_side if last_side != 0 else +1
            # Arc in the chosen direction (no reverse)
            if side > 0:
                # turn RIGHT: left > right
                cmdL = SEARCH_FWD + SEARCH_TURN
                cmdR = SEARCH_FWD - SEARCH_TURN
            else:
                # turn LEFT: right > left
                cmdL = SEARCH_FWD - SEARCH_TURN
                cmdR = SEARCH_FWD + SEARCH_TURN

            set_motors(cmdL, cmdR)

            # Reacquire conditions: any sensor sees enough darkness or center close to line
            if (wL >= 0.20) or (wC >= 0.20) or (wR >= 0.20) or (Cr <= CENTER_ON_LINE):
                state = FOLLOW
                cooldown_t = REACQ_COOLDOWN
                lost_t = 0.0
                snd.speak("Line reacquired")
                leds.set_color('LEFT','GREEN'); leds.set_color('RIGHT','GREEN')

            # If too long, flip side to avoid spinning forever the wrong way
            elif (t_now - search_t0) >= SEARCH_SIDE_SWITCH:
                last_side = -side
                search_t0 = t_now
                snd.speak("Switch side")

            pos = 0.0; base = 0.0; steer = (cmdL - cmdR)
            state_name = "SEARCH"

        # Log
        writer.writerow([
            round(t_now - t0,3),
            int(Lr), int(Cr), int(Rr),
            round(L,3), round(C,3), round(R,3),
            round(wL,3), round(wC,3), round(wR,3),
            round(pos,3), round(base,1), round(steer,1),
            int(lm.speed), int(rm.speed),
            state_name
        ])
        f.flush()

        # Fixed 50 Hz loop
        loop_dt = time.time() - t_now
        if loop_dt < 0.02:
            time.sleep(0.02 - loop_dt)

except KeyboardInterrupt:
    pass
finally:
    lm.stop(); rm.stop()
    leds.all_off()
    snd.speak("Mission complete.")
    f.close()
    print("CSV:", csv_path)
