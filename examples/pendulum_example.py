"""
Standing pendulum example for the Lite 6 simulator/robot.

The arm oscillates around a neutral upright pose with a smooth sinusoid
on shoulder (J2) and elbow (J3), mimicking a standing pendulum swing.
Safe limits and conservative speed/accel are used by default.
"""
import math
import time

import config
from xarm.wrapper import XArmAPI  # shimmed by LiteSim's GUI to SimXArmAPI

# --- Tunables ---
SWING_AMPLITUDE_J2 = 15.0   # deg peak (shoulder)
SWING_AMPLITUDE_J3 = 25.0   # deg peak (elbow)
SWING_FREQUENCY_HZ = 0.25   # cycles per second
BASE_SPEED_DEG_S = 60.0     # joint speed target (clamped by robot_api)
BASE_ACC_DEG_S2 = 200.0     # joint accel target (clamped by robot_api)
RUN_TIME_SEC = 30.0         # total demo duration

# Neutral upright pose (all zeros here; adjust if you prefer a different center)
NEUTRAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def main(ip=None):
    # In the GUI this will be replaced by the SimXArmAPI instance automatically.
    arm = XArmAPI(ip or "127.0.0.1")

    # Move to neutral first
    arm.set_servo_angle(NEUTRAL, speed=BASE_SPEED_DEG_S, mvacc=BASE_ACC_DEG_S2, is_radian=False, wait=True)

    t0 = time.time()
    try:
        while True:
            now = time.time()
            if RUN_TIME_SEC and (now - t0) > RUN_TIME_SEC:
                break
            phase = 2.0 * math.pi * SWING_FREQUENCY_HZ * (now - t0)
            j2 = NEUTRAL[1] + SWING_AMPLITUDE_J2 * math.sin(phase)
            j3 = NEUTRAL[2] + SWING_AMPLITUDE_J3 * math.sin(phase)

            # Respect configured joint limits
            j2 = clamp(j2, config.JOINT_LIMITS[1][0], config.JOINT_LIMITS[1][1])
            j3 = clamp(j3, config.JOINT_LIMITS[2][0], config.JOINT_LIMITS[2][1])

            targets = [NEUTRAL[0], j2, j3, NEUTRAL[3], NEUTRAL[4], NEUTRAL[5]]
            arm.set_servo_angle(targets, speed=BASE_SPEED_DEG_S, mvacc=BASE_ACC_DEG_S2, is_radian=False, wait=False)
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        # Park back at neutral
        arm.set_servo_angle(NEUTRAL, speed=BASE_SPEED_DEG_S, mvacc=BASE_ACC_DEG_S2, is_radian=False, wait=True)


if __name__ == "__main__":
    main()
