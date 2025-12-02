"""
Standing-stick pendulum wave demo.

Pose: arm fully upright with J3 = 180 deg, all other joints = 0.
Motion: small sinusoidal offsets with slightly different frequencies per joint
to create a pendulum wave effect while keeping the stick mostly vertical.
"""
import math
import time

import config
from xarm.wrapper import XArmAPI  # shimmed to SimXArmAPI inside the GUI

# --- Tunables ---
BASE_POSE = [0.0, 0.0, 180.0, 0.0, 0.0, 0.0]
DURATION_SEC = 30.0
BASE_SPEED = 90.0    # deg/s (moderate base speed)
BASE_ACC = 220.0     # deg/s^2
DT = 0.02            # smooth refresh
PRE_DELAY = 2.0      # seconds before starting motion
RAMP_TIME = 10.0     # slower ramp in/out
DURATION_SEC = 50.0  # total run time including ramp

# Larger amplitudes for a bolder wave while keeping motion smooth
# (J3 swings around its 180° base pose)
AMPLITUDES = [40.0, 32.0, 18.0, 26.0, 24.0, 20.0]
# Frequencies tuned for a rolling wave (overall slower sway across all joints)
FREQS = [0.4, 0.5, 0.55, 0.6, 0.65, 0.7]


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def main(ip=None):
    arm = XArmAPI(ip or "127.0.0.1")

    # Move to upright base pose
    arm.set_servo_angle(BASE_POSE, speed=BASE_SPEED, mvacc=BASE_ACC, wait=True, is_radian=False)

    t_start = time.time()
    try:
        while True:
            now = time.time()
            elapsed = now - t_start

            # Pre-delay before motion starts
            if elapsed < PRE_DELAY:
                time.sleep(DT)
                continue

            wave_t = elapsed - PRE_DELAY
            if DURATION_SEC and wave_t > DURATION_SEC:
                break

            # Envelope for smooth ramp up/down (smoothstep on min of in/out ramps)
            ramp_in = min(1.0, wave_t / RAMP_TIME)
            ramp_out = 1.0
            if DURATION_SEC > 0:
                ramp_out = min(1.0, (DURATION_SEC - wave_t) / RAMP_TIME)
            env = min(ramp_in, ramp_out)
            # smoothstep
            # smootherstep for softer edges
            env = env * env * env * (env * (6 * env - 15) + 10)

            offsets = []
            for i in range(6):
                if FREQS[i] == 0.0:
                    offsets.append(0.0)
                    continue
                phase = 2 * math.pi * FREQS[i] * wave_t
                offsets.append(AMPLITUDES[i] * env * math.sin(phase))

            targets = [BASE_POSE[i] + offsets[i] for i in range(6)]

            # Respect configured limits
            for i, (lo, hi) in enumerate(config.JOINT_LIMITS):
                targets[i] = clamp(targets[i], lo, hi)

            # Slightly modulate speed with envelope for a gentle ramp feel
            effective_speed = BASE_SPEED * (0.6 + 0.4 * env)
            effective_acc = BASE_ACC * (0.6 + 0.4 * env)

            arm.set_servo_angle(targets, speed=effective_speed, mvacc=effective_acc, wait=False, is_radian=False)
            time.sleep(DT)
    except KeyboardInterrupt:
        pass
    finally:
        # Return to upright
        arm.set_servo_angle(BASE_POSE, speed=BASE_SPEED, mvacc=BASE_ACC, wait=True, is_radian=False)


if __name__ == "__main__":
    main()
