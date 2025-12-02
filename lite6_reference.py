"""
UFACTORY Lite 6 – condensed reference for emulation/simulation.

Source docs:
- Lite6_Hardware_Manual_V2.6.0.pdf :contentReference[oaicite:0]{index=0}
- Lite-6-Developer-Manual-V1.11.0.pdf :contentReference[oaicite:1]{index=1}
- Kinematic-and-Dynamic-Parameters-of-UFACTORY-Lite-6.pdf :contentReference[oaicite:2]{index=2}
"""

# ---------------------------
# Units & conventions
# ---------------------------

LITE6_UNITS = {
    "cartesian_pos": "mm",          # x,y,z in millimeters
    "angles_sdk": "deg",            # roll/pitch/yaw & J1..J6 in Python SDK / Blockly
    "angles_protocol": "rad",       # roll/pitch/yaw & J1..J6 in TCP private protocol
    "tcp_speed": "mm/s",
    "tcp_acc": "mm/s^2",
    "tcp_jerk": "mm/s^3",
    "joint_speed_sdk": "deg/s",     # protocol: rad/s
    "joint_acc_sdk": "deg/s^2",     # protocol: rad/s^2
    "joint_jerk_sdk": "deg/s^3",    # protocol: rad/s^3
}

# Coordinate systems (high level, for docstrings / comments)
COORDINATE_SYSTEMS = {
    "base": "Cartesian frame attached to robot base: X forward, Y left, Z up.",
    "tool": (
        "Tool coordinate system at tool center point (TCP). "
        "If TCP offset is zero, TCP is at flange center."
    ),
    "user": "User-defined frame relative to base (workpiece, fixture, etc.).",
    "rpy_definition": (
        "Roll/Pitch/Yaw are XYZ fixed angles: rotate about base X (roll=γ), "
        "then base Y (pitch=β), then base Z (yaw=α)."
    ),
    "axis_angle": (
        "Axis-angle pose represented as [Rx,Ry,Rz] = unit_axis * angle(rad). "
        "Vector direction = rotation axis; vector norm = rotation angle."
    ),
}

# ---------------------------
# Joint limits & motion limits
# ---------------------------

JOINT_LIMITS_DEG = {
    "J1": (-360.0, 360.0),
    "J2": (-150.0, 150.0),
    "J3": (-3.5, 300.0),
    "J4": (-360.0, 360.0),
    "J5": (-124.0, 124.0),
    "J6": (-360.0, 360.0),
}

MOTION_LIMITS = {
    "tcp_speed_mm_s": (0.0, 500.0),
    "tcp_acc_mm_s2": (0.0, 50000.0),
    "tcp_jerk_mm_s3": (0.0, 100000.0),
    "joint_speed_deg_s": (0.0, 180.0),
    "joint_acc_deg_s2": (0.0, 1145.0),
    "joint_jerk_deg_s3": (0.0, 28647.0),
}

# ---------------------------
# Kinematics – DH parameters
# ---------------------------
# Values from “Kinematic and Dynamic Parameters of UFACTORY Lite 6”.
# Distance units: mm, angles: deg. You can convert to radians at runtime.

# Modified DH parameters (commonly used in robotics libraries)
MDH_TABLE = [
    # joint,  theta_offset[deg],  d[mm],   alpha[deg],  a[mm]
    ("J1",    0.0,                243.3,   0.0,         0.0),
    ("J2",   -90.0,               0.0,    -90.0,        0.0),
    ("J3",   -90.0,               0.0,    180.0,        200.0),
    ("J4",    0.0,                227.6,  90.0,         87.0),
    ("J5",    0.0,                0.0,    90.0,         0.0),
    ("J6",    0.0,                61.5,   -90.0,        0.0),
]

# Standard DH parameters (alternate convention)
SDH_TABLE = [
    # joint,  theta_offset[deg],  d[mm],   alpha[deg],  a[mm]
    ("J1",    0.0,                243.3,  -90.0,        0.0),
    ("J2",   -90.0,               0.0,    180.0,        200.0),
    ("J3",   -90.0,               0.0,    90.0,         87.0),
    ("J4",    0.0,                227.6,  90.0,         0.0),
    ("J5",    0.0,                0.0,    -90.0,        0.0),
    ("J6",    0.0,                61.5,   0.0,          0.0),
]

# ---------------------------
# Dynamics – link masses & centers of mass
# ---------------------------

LINK_MASS_AND_COM = [
    # name,  mass_kg,                com_xyz_mm (in link frame)
    ("Link1", 1.411, [-0.36,  41.95,  -2.5]),
    ("Link2", 1.340, [179.0,   0.0,   58.4]),
    ("Link3", 0.953, [72.0,  -35.7,   -1.0]),
    ("Link4", 1.284, [-2.0,  -28.5,  -81.3]),
    ("Link5", 0.804, [0.0,   10.0,     1.9]),
    ("Link6", 0.130, [0.0,   -1.94,  -10.2]),
]

# ---------------------------
# Communication protocol (TCP private protocol)
# ---------------------------

PROTOCOL = {
    "tcp_port": 502,
    "protocol_id": 0x0002,  # private control protocol
    "request_header": [
        # (name,     type, bytes, endian, notes)
        ("transaction_id", "u16", 2, "big", "increment per packet; echoed in response"),
        ("protocol",       "u16", 2, "big", "0x0002 = private protocol"),
        ("length",         "u16", 2, "big", "bytes following (register + params)"),
        ("register",       "u8",  1, "n/a", "command/register id"),
        # params follow (see registers)
    ],
    "response_header": [
        ("transaction_id", "u16", 2, "big"),
        ("protocol",       "u16", 2, "big"),
        ("length",         "u16", 2, "big"),
        ("register",       "u8",  1, "n/a"),
        ("status",         "u8",  1, "bit flags: error/warning/etc."),
        # params follow
    ],
    "status_bits": {
        "bit7": "1 = error",
        "bit6": "1 = warning",
        "bit5": "1 = cannot perform motion",
        # remaining bits currently unused or reserved
    },
    "endianness_rules": {
        "header_fields": "u16 in header are big endian",
        "fp32_params": "little endian (IEEE-754)",
        "gpio_u16": "big endian in GPIO-related regs",
        "auto_report_ints": "16/32-bit ints big endian; fp32 little endian",
    },
}

# ---------------------------
# Key registers for emulation
# (subset of full map; enough for a realistic emulator)
# ---------------------------

REG = {
    # 0–10: common
    "GET_VERSION": 0x01,            # request: none, response: state + version
    "GET_SN": 0x02,                 # response includes robot+controller serial string
    "RELOAD_FRICTION": 0x04,
    "GET_TORQUE_OR_CURRENT": 0x05,  # param selects theoretical torque vs actual current
    "GET_JOINT_TCP_RADIUS": 0x06,
    "REMOTE_SHUTDOWN_OS": 0x0A,

    # 11–20: system state
    "SET_SERVO_ENABLE": 0x0B,       # params: joint selector (1–6 or 8=all), enable(1/0)
    "SET_MOTION_STATE": 0x0C,       # 0=motion mode, 3=suspend, 4=stop, etc.
    "GET_MOTION_STATE": 0x0D,       # returns current motion state enum
    "GET_CMD_BUFFER_SIZE": 0x0E,    # number of queued commands
    "GET_ERROR_WARNING": 0x0F,      # returns error_code, warning_code
    "CLEAR_ERRORS": 0x10,           # system reset side effect
    "CLEAR_WARNINGS": 0x11,
    "SET_BRAKE": 0x12,              # select joint(s), enable(1)/release(0) – system reset
    "SET_MOTION_MODE": 0x13,        # 0=position, 1=servo, 2=joint teach, 3=cartesian teach

    # 21–30: basic motion
    "MOVE_L": 0x15,                 # Cartesian linear move (x,y,z,rpy,speed,acc,time)
    "MOVE_L_ARC_BLEND": 0x16,       # linear with circular arc blending
    "MOVE_J": 0x17,                 # joint P2P, params: 6 joint targets + speed/acc/time
    "MOVE_J_ARC_BLEND": 0x18,
    "MOVE_HOME": 0x19,              # return to zero position
    "DELAY_OR_PAUSE": 0x1A,
    "MOVE_C": 0x1B,                 # circular Cartesian motion
    "MOVE_L_TOOL_FRAME": 0x1C,
    "SERVOJ": 0x1D,                 # joint-space servo
    "SERVO_CARTESIAN": 0x1E,

    # 31–40: system parameter setting
    "SET_TCP_JERK": 0x1F,
    "SET_TCP_MAX_ACC": 0x20,
    "SET_JOINT_JERK": 0x21,
    "SET_JOINT_MAX_ACC": 0x22,
    "SET_TCP_OFFSET": 0x23,         # tool center point offset; system reset
    "SET_PAYLOAD": 0x24,            # payload mass + CoM
    "SET_COLLISION_SENS": 0x25,     # 0–5, 0 = disabled
    "SET_TEACH_SENS": 0x26,         # 1–5
    "DELETE_CONFIG": 0x27,          # delete current system configuration
    "SAVE_CONFIG": 0x28,

    # 41–50: get motion info
    "GET_TCP_POSE": 0x29,           # returns x,y,z,roll,pitch,yaw (or axis-angle)
    "GET_JOINT_POS": 0x2A,          # returns current joint angles
    "IK_SOLVE": 0x2B,               # inverse kinematics
    "FK_SOLVE": 0x2C,               # forward kinematics
    "CHECK_JOINT_LIMIT": 0x2D,
    "SET_REDUCED_TCP_SPEED_LIMIT": 0x2F,
    "SET_REDUCED_JOINT_SPEED_LIMIT": 0x30,
    "GET_REDUCED_MODE_STATE": 0x31,
    "SET_REDUCED_MODE_STATE": 0x32,

    # 51–100: other functions (subset)
    "SET_GRAVITY_DIR": 0x33,
    "SET_SAFETY_BOUNDARY": 0x34,
    "GET_REDUCED_MODE_CONFIG": 0x35,
    "GET_JOINT_TORQUE": 0x37,
    "SET_REDUCED_JOINT_RANGE": 0x3A,
    "SET_SAFETY_BOUNDARY_ENABLE": 0x3B,
    "SET_COLLISION_REBOUND": 0x3C,
    "TRAJ_RECORD_START_STOP": 0x3D,
    "TRAJ_RECORD_SAVE": 0x3E,
    "TRAJ_RECORD_LOAD": 0x3F,
    "TRAJ_RECORD_PLAY": 0x40,
    "TRAJ_RECORD_STATE": 0x41,
    "SET_ALLOW_SINGULAR_OVERSPEED_APPROX": 0x42,
    "SET_JOINT_TORQUE_CURRENT_THEORETICAL": 0x46,
    "SET_USER_FRAME_OFFSET": 0x49,
    "CALC_ATTITUDE_OFFSET": 0x4C,
    "SET_SELF_COLLISION": 0x4D,
    "SET_TOOL_GEOMETRY_FOR_COLLISION": 0x4E,
    "SET_VIRTUAL_ROBOT": 0x4F,      # enable virtual-arm mode
    "SET_CARTESIAN_VEL_CONTINUOUS": 0x50,
    "JOINT_VEL_CONTROL": 0x51,
    "CARTESIAN_VEL_CONTROL": 0x52,
    "RELATIVE_MOTION_CONTROL": 0x53,
    "GET_AXIS_ANGLE_FROM_RPY": 0x5B,
    "MOVE_L_AXIS_ANGLE": 0x5C,
    "SERVO_CARTESIAN_AXIS_ANGLE": 0x5D,

    # 101–115: servo / module
    "GET_SERVO_STATE": 0x6A,

    # 145: IO-related example
    "IO_ACTION_AT_POSITION": 0x91,  # triggers digital IO change when TCP enters tol sphere
}

# Example: helper describing motion state codes (used by GET_MOTION_STATE)
MOTION_STATE_ENUM = {
    1: "in_motion",
    2: "sleep",
    3: "suspend",
    4: "stop",
    5: "system_reset",  # after mode change or some settings; movement cleared
}

# ---------------------------
# Convenience: high-level “spec” object
# ---------------------------

LITE6_SPEC = {
    "joint_limits_deg": JOINT_LIMITS_DEG,
    "motion_limits": MOTION_LIMITS,
    "mdh_table": MDH_TABLE,
    "sdh_table": SDH_TABLE,
    "link_mass_and_com": LINK_MASS_AND_COM,
    "units": LITE6_UNITS,
    "coordinate_systems": COORDINATE_SYSTEMS,
    "protocol": PROTOCOL,
    "registers": REG,
    "motion_state_enum": MOTION_STATE_ENUM,
}
