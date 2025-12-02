# LiteSim (fork, customized)

Simulator and controller for the UFACTORY Lite 6 arm, heavily customized. Run in simulation or connect to real hardware, visualize motions in 3D, and drive scripts from a single Qt UI.

![Python](https://img.shields.io/badge/Python-3.10+-blue.svg?logo=python&logoColor=white)

## Highlights
- Unified Qt UI with embedded PyVista viewer; camera controls and restart in the header.
- Sim-only safety toggle to block real-arm commands even when connected.
- Collision handling: halts motion and offers Resume / Reset to Home / Save Snapshot.
- Preflight checks: parses scripts for out-of-limit joint/TCP speeds/positions before running.
- Real or sim: connect to a Lite 6 (xarm-python-sdk) or stay in sim; stream joints over TCP.
- Script runner: load recent/example scripts, loop, pause, restart; motion demos included.
- Customization: color presets, ghost mode, trace source, end-effector STL loading.

## Requirements
- Python 3.10+
- Dependencies: see `requirements.txt`
- Optional for real arm: `xarm-python-sdk`

## Quick Start
```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python gui_qt.py
```

### Using the Qt GUI
- Robot Connection: enter IP, toggle Sim-only, Connect/Disconnect.
- Script Loading: pick recent/example scripts, refresh list, start/stop/pause/loop.
- Live Stream: start listener on host/port (default 127.0.0.1:7777) for external joint feeds.
- Manual Joints: sliders/spinboxes with enforced limits; Reset to Home/View.
- Collision: on impact, choose Resume, Reset to Home, or Save Snapshot (JSON in `~/.litesim`).
- Colors/Trace: adjust colors, presets, trace source, ghost mode.

### Examples
In `examples/` you’ll find:
- Pendulum demos (`pendulum_example.py`, `pendulum_wave_stick.py`)
- Breathing / curiosity / swaying / nod / heartbeat / gliding motions
- Basic moves (`move_basic.py`, `move_wave.py`, etc.)

## Credits
- Current maintainer: Rochee Faverey ([Unskilled_Skill](https://github.com/Unskilled_Skill))
- Original author: Dylan Kiesebrink ([scoopy115](https://github.com/scoopy115))
- SDK: xArm-Python-SDK
- Visualization: PyVista
- IK: IKPy

## Disclaimer
Not affiliated with UFACTORY. 3D models (`.urdf`, `.stl`) and Lite 6 design are the property of UFACTORY. Use responsibly and at your own risk when controlling real hardware.

## License
MIT — see `LICENSE`.
