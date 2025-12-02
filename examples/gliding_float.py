"""
Gliding / floating motion: smooth looping arcs using sine/cosine pairs for a weightless feel.
"""
import math
import time

import config
from xarm.wrapper import XArmAPI  # shimmed to SimXArmAPI by the GUI

BASE_POSE = [0.0, 0.0, 180.0, 0.0, 0.0, 0.0]
SPEED = 85.0
ACC = 180.0
DT = 0.02
PRE_DELAY = 1.0
DURATION = 50.0
RAMP = 8.0

FREQ = 0.32  # Hz base frequency for glide

def smootherstep(x):
    return x * x * x * (x * (6 * x - 15) + 10)

def envelope(t):
    if DURATION <= 0:
        return 1.0
    edge = min(RAMP, DURATION * 0.4)
    ramp_in = min(1.0, max(0.0, t / edge))
    ramp_out = min(1.0, max(0.0, (DURATION - t) / edge))
    return smootherstep(min(ramp_in, ramp_out))

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def main(ip=None):
    arm = XArmAPI(ip or "127.0.0.1")
    arm.set_servo_angle(BASE_POSE, speed=SPEED, mvacc=ACC, wait=True, is_radian=False)
    t0 = time.time()

    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed < PRE_DELAY:
                time.sleep(DT)
                continue
            t = elapsed - PRE_DELAY
            if DURATION and t > DURATION:
                break

            env = envelope(t)
            j2 = 12.0 * math.sin(2 * math.pi * FREQ * t)
            j3 = 10.0 * math.cos(2 * math.pi * FREQ * t)
            j4 = 8.0 * math.sin(2 * math.pi * (FREQ * 0.92) * t + 0.6)

            targets = [
                BASE_POSE[0],
                BASE_POSE[1] + env * j2,
                BASE_POSE[2] + env * j3,
                BASE_POSE[3] + env * j4,
                BASE_POSE[4],
                BASE_POSE[5],
            ]

            for i, (lo, hi) in enumerate(config.JOINT_LIMITS):
                targets[i] = clamp(targets[i], lo, hi)

            spd = SPEED * (0.6 + 0.4 * env)
            acc = ACC * (0.6 + 0.4 * env)
            arm.set_servo_angle(targets, speed=spd, mvacc=acc, wait=False, is_radian=False)
            time.sleep(DT)
    finally:
        arm.set_servo_angle(BASE_POSE, speed=SPEED, mvacc=ACC, wait=True, is_radian=False)

if __name__ == "__main__":
    main()
