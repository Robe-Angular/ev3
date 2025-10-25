# file: claw_keyboard_grip_safe.py
# keyboard control with live load logging, grip detection, and safe interrupt (ascii only)

from ev3dev2.motor import MediumMotor, OUTPUT_A
from ev3dev2.power import PowerSupply
from time import sleep, time
import sys, termios, tty, signal

# config
PORT = OUTPUT_A
SPEED = 20
SAMPLE_DT = 0.05
STEP_DEG = 40
SAMPLE_DT = 0.05
STALL_SPEED = 30
STALL_DUTY = 45
HIT_COUNT = 6
LOG_PATH = "/home/robot/claw_grip_log.csv"
# position freeze detector
POS_EPS = 2         # deg: position change considered "stopped"
POS_HITS = 5        # samples in a row with tiny motion

m = MediumMotor(PORT)
ps = PowerSupply()

# --- emergency stop on Ctrl+C ---
def stop_all(sig=None, frame=None):
    print("\nEmergency stop")
    m.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, stop_all)

def log_header():
    f = open(LOG_PATH, "w")
    f.write("time_s,action,position,speed,duty,voltage,current\n")
    f.close()

def log_line(action):
    try: v = ps.measured_volts
    except: v = 0.0
    try: c = ps.measured_amps
    except: c = 0.0
    t = time()
    line = "%.3f,%s,%d,%d,%d,%.3f,%.3f\n" % (t, action, m.position, m.speed, m.duty_cycle, v, c)
    f = open(LOG_PATH, "a"); f.write(line); f.close()

def sample_loop(label, until_cond=None):
    while "running" in m.state:
        log_line(label)
        if until_cond and until_cond():
            break
        sleep(SAMPLE_DT)
    log_line(label + "_end")

def close_until_grip():
    print("closing until grip...")
    m.on(-SPEED)  # invert direction so it closes correctly
    hits = 0
    pos_hits = 0
    last_pos = m.position
    def cond():
        nonlocal hits, pos_hits, last_pos
        spd = abs(m.speed)
        dty = abs(m.duty_cycle)
        if spd < STALL_SPEED and dty > STALL_DUTY:
            hits += 1
        else:
            hits = 0
        return hits >= HIT_COUNT
    
    sample_loop("close_hold", cond)
    m.stop()
    log_line("close_hold_stop")


    if hits >= HIT_COUNT or pos_hits >= POS_HITS:
        print("grip detected")
    else:
        print("stopped")

def open_short():
    print("opening step")
    m.on_for_degrees(speed=SPEED, degrees=STEP_DEG)
    log_line("open_step_end")

def close_short():
    print("closing step")
    m.on_for_degrees(speed=-SPEED, degrees=STEP_DEG)
    log_line("close_step_end")

def open_burst():
    print("opening burst")
    m.on(SPEED)
    t0 = time()
    while time() - t0 < 0.5:
        log_line("open_burst")
        sleep(SAMPLE_DT)
    m.stop()
    log_line("open_burst_stop")

def kb_read_char():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    print("keyboard control ready")
    print("a=close until grip | z=open burst | s=close step | x=open step | q=quit (Ctrl+C also works)")
    log_header()
    while True:
        key = kb_read_char()
        if key == 'a':
            close_until_grip()
        elif key == 'z':
            open_burst()
        elif key == 's':
            close_short()
        elif key == 'x':
            open_short()
        elif key == 'q':
            stop_all()
        sleep(0.02)

if __name__ == "__main__":
    main()
