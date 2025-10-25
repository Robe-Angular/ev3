# file: claw_load_logger.py
# ev3dev2: log motor load while testing the claw

from ev3dev2.motor import MediumMotor, OUTPUT_A
from ev3dev2.button import Button
from ev3dev2.power import PowerSupply
from time import sleep, time

# config
PORT = OUTPUT_A
STEP_DEG = 60           # move per button tap (up/down)
SPEED = 30              # speed percent
SAMPLE_DT = 0.05        # seconds between samples
STALL_SPEED = 5         # tacho speed close to zero
STALL_DUTY = 85         # pwm duty high -> motor pushing hard
LOG_PATH = "/home/robot/garra_log.csv"

m = MediumMotor(PORT)
btn = Button()
ps = PowerSupply()

def log_header():
    with open(LOG_PATH, "w") as f:
        f.write("time_s,action,position,speed,duty,voltage,current\n")

def log_line(action):
    # voltage_now in mV, current_now in mA
    try:
        v = ps.measured_volts
    except:
        v = 0.0
    try:
        c = ps.measured_amps
    except:
        c = 0.0
    t = time()
    line = f"{t:.3f},{action},{m.position},{m.speed},{m.duty_cycle},{v:.3f},{c:.3f}\n"
    with open(LOG_PATH, "a") as f:
        f.write(line)

def sample_while_running(action_label):
    # sample motor telemetry until it stops running
    while "running" in m.state:
        log_line(action_label)
        sleep(SAMPLE_DT)
    # one last sample after stop
    log_line(action_label + "_end")

def move_rel_logged(deg, label):
    m.run_to_rel_pos(position_sp=deg, speed_sp=SPEED, stop_action="brake")
    sample_while_running(label)

def run_until_stall(direction, label):
    # direction: +1 close, -1 open
    m.on(direction * SPEED)
    # sample until stall
    stall_count = 0
    while True:
        log_line(label)
        spd = abs(m.speed)
        dty = abs(m.duty_cycle)
        if spd < STALL_SPEED and dty > STALL_DUTY:
            stall_count += 1
        else:
            stall_count = 0
        if stall_count >= 5:   # ~5 samples -> ~0.25 s stable stall
            break
        # allow user to abort with center button
        if btn.enter:
            break
        sleep(SAMPLE_DT)
    m.stop()
    log_line(label + "_stall_stop")

def main():
    print("ready. DOWN=close step, UP=open step, LEFT=close until stall, RIGHT=open until stall, CENTER=exit")
    log_header()
    while True:
        if btn.down:
            move_rel_logged(+STEP_DEG, "close_step")
            sleep(0.2)
        elif btn.up:
            move_rel_logged(-STEP_DEG, "open_step")
            sleep(0.2)
        elif btn.left:
            run_until_stall(+1, "close_hold")
            sleep(0.2)
        elif btn.right:
            run_until_stall(-1, "open_hold")
            sleep(0.2)
        elif btn.enter:
            print("exit")
            m.stop()
            break
        sleep(0.05)

if __name__ == "__main__":
    main()
