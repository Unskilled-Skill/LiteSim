# config.py
import sys

# --- APP INFO ---
APP_NAME = "LiteSim"
APP_VERSION = "1.9.3"
AUTHOR = "Dylan Kiesebrink"
GITHUB_URL = "https://github.com/scoopy115/LiteSim"
PORTFOLIO_URL = "https://dylankiesebrink.nl"

# --- ROBOT CONFIGURATION ---
JOINT_COUNT = 6
ROBOT_Z_OFFSET = -0.19
SIM_SPEED_FACTOR = 1.0 
ROBOT_SCAN_PORT = 30002

# --- GLOBAL API REFERENCE ---
GLOBAL_API_INSTANCE = None 

# --- XARM SDK CHECK ---
try:
    from xarm.wrapper import XArmAPI as RealXArmAPI
    HAS_REAL_SDK = True
    print("[SYSTEM] xarm-python-sdk found! Working in simulation and robot mode.")
except ImportError:
    HAS_REAL_SDK = False
    print("[SYSTEM] xarm-python-sdk not found. Working in simulation-mode only.")

# --- FILE PATHS ---
HISTORY_FILE = "recent_scripts.txt" 
STL_HISTORY_FILE = "recent_stls.txt"

# --- WINDOW SETTINGS ---
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 1000
INITIAL_ZOOM_LEVEL = 2