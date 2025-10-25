# file: claw_keyboard_grip.py
# keyboard control with live load logging and grip detection (ascii only)

from ev3dev2.motor import MediumMotor, OUTPUT_A
from ev3dev2.power import PowerSupply
from time import sleep, time
import sys, termios, tty

# config
PORT = OUTPUT_A
SPEED = 30            # motor speed percent for continuous moves
STEP_DEG = 40         # degrees for step moves (s/x)
SAMPLE_DT = 0.05      # seconds between samples
STALL_SPEED = 5       # tacho speed threshold (abs) for "almost stopped"
STALL_DUTY = 80       # duty threshold (abs) for "pushing hard"
HIT_COUNT = 6         # samples in a row to confirm grip (~0.3 s)
LOG_PATH = "/home/robot/claw_grip_log.csv"

m = MediumMotor(PORT)
ps = PowerSupply()

def log_header():
    f = open(LOG_PATH, "w")
    f.write("time_s,action,position,speed,duty,voltage,current\n")
    f.close()

def log_line(action):
    # power supply readings may not be available on all images
    try:
        v = ps.measured_volts
    except:
        v = 0.0
    try:
        c = ps.measured_amps
    except:
        c = 0.0
    t = time()
    line = "%.3f,%s,%d,%d,%d,%.3f,%.3f\n" % (t, action, m.position, m.speed, m.duty_cycle, v, c)
    f = open(LOG_PATH, "a")
    f.write(line)
    f.close()

def sample_loop(label, until_cond=None):
    # sample telemetry until motor stops or condition returns True
    while "running" in m.state:
        log_line(label)
        if until_cond is not None and until_cond():
            break
        sleep(SAMPLE_DT)
    # final sample
    log_line(label + "_end")

def close_until_grip():
    print("close until grip...")
    m.on(SPEED)
    hits = 0
    def cond():
        nonlocal hits
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
    if hits >= HIT_COUNT:
        print("grip detected")
    else:
        print("stopped")

def open_short():
    print("open step")
    m.on_for_degrees(speed=-SPEED, degrees=STEP_DEG)
    log_line("open_step_end")

def close_short():
    print("close step")
    m.on_for_degrees(speed=SPEED, degrees=STEP_DEG)
    log_line("close_step_end")

def open_burst():
    print("open burst")
    m.on(-SPEED)
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
    print("a=close until grip | z=open burst | s=close step | x=open step | q=quit")
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
            print("quit")
            m.stop()
            break
        sleep(0.02)

if __name__ == "__main__":
    main()
