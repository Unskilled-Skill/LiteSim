"""
Curiosity lean / head-tilt: slow inquisitive motion with a touch of randomness.
J1 and J4 tilt, J2 adds a tiny lean to feel attentive.
"""
import math
import time
import random

import config
from xarm.wrapper import XArmAPI  # shimmed to SimXArmAPI by the GUI

BASE_POSE = [0.0, 0.0, 180.0, 0.0, 0.0, 0.0]
SPEED = 80.0
ACC = 180.0
DT = 0.02
PRE_DELAY = 1.0
DURATION = 45.0
RAMP = 6.0

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

    phase_j1 = random.uniform(0, math.pi)
    phase_j4 = random.uniform(0, math.pi)
    lean_phase = random.uniform(0, math.pi)

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
            j1 = 12.0 * math.sin(2 * math.pi * 0.22 * t + phase_j1)
            j4 = 8.0 * math.sin(2 * math.pi * 0.30 * t + phase_j4)
            j2 = 3.0 * math.sin(2 * math.pi * 0.18 * t + lean_phase)

            targets = [j1, j2, BASE_POSE[2], j4, 0.0, 0.0]
            targets = [BASE_POSE[i] + env * (targets[i] - BASE_POSE[i]) for i in range(6)]

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
