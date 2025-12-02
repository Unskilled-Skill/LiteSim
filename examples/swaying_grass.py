"""
Swaying / grass-in-the-wind motion: staggered joints with close-but-not-equal frequencies.
Adds a phase offset per joint for a cascading, organic sway.
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

AMPLITUDES = [6.0, 10.0, 14.0, 6.0, 3.0, 0.0]
FREQS = [0.10, 0.15, 0.17, 0.20, 0.22, 0.0]
PHASE_STEP = 0.45  # per-joint phase lag

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
            offsets = []
            for i in range(6):
                if FREQS[i] == 0.0:
                    offsets.append(0.0)
                else:
                    phase = i * PHASE_STEP
                    offsets.append(AMPLITUDES[i] * math.sin(2 * math.pi * FREQS[i] * t + phase))

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
