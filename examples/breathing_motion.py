"""
Gentle breathing motion: subtle, organic oscillations that make the arm feel alive.
J2–J4 carry most of the motion with slightly different slow frequencies.
"""
import math
import time
import random

import config
from xarm.wrapper import XArmAPI  # shimmed to SimXArmAPI by the GUI

BASE_POSE = [0.0, 0.0, 180.0, 0.0, 0.0, 0.0]
SPEED = 70.0
ACC = 160.0
DT = 0.02
PRE_DELAY = 1.5
DURATION = 60.0
RAMP = 8.0  # seconds for ease-in/out

# Amplitudes and frequencies (Hz)
AMPLITUDES = [0.0, 3.0, 5.0, 4.0, 1.5, 0.0]
FREQS = [0.0, 0.11, 0.13, 0.10, 0.14, 0.0]


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

    # slight random phase offsets to avoid perfect sync
    phases = [random.uniform(0, math.pi) for _ in range(6)]

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
            offsets = []
            for i in range(6):
                if FREQS[i] == 0:
                    offsets.append(0.0)
                else:
                    offsets.append(AMPLITUDES[i] * math.sin(2 * math.pi * FREQS[i] * t + phases[i]))

            targets = [BASE_POSE[i] + env * offsets[i] for i in range(6)]
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
