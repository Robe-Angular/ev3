# file: claw_load_logger.py
# ev3dev2: log motor load while testing the claw (ascii only)

from ev3dev2.motor import MediumMotor, OUTPUT_A
from ev3dev2.button import Button
from ev3dev2.power import PowerSupply
from time import sleep, time

PORT = OUTPUT_A
STEP_DEG = 60
SPEED = 30
SAMPLE_DT = 0.05
STALL_SPEED = 5
STALL_DUTY = 85
LOG_PATH = "/home/robot/garra_log.csv"

m = MediumMotor(PORT)
btn = Button()
ps = PowerSupply()

def log_header():
    f = open(LOG_PATH, "w")
    f.write("time_s,action,position,speed,duty,voltage,current\n")
    f.close()

def log_line(action):
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

def sample_while_running(action_label):
    while "running" in m.state:
        log_line(action_label)
        sleep(SAMPLE_DT)
    log_line(action_label + "_end")

def move_rel_logged(deg, label):
    m.run_to_rel_pos(position_sp=deg, speed_sp=SPEED, stop_action="brake")
    sample_while_running(label)

def run_until_stall(direction, label):
    m.on(direction * SPEED)
    stall_count = 0
    while True:
        log_line(label)
        spd = abs(m.speed)
        dty = abs(m.duty_cycle)
        if spd < STALL_SPEED and dty > STALL_DUTY:
            stall_count += 1
        else:
            stall_count = 0
        if stall_count >= 5:
            break
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
