# LiteSim: UFACTORY Lite 6 Simulator & Controller

![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)

**LiteSim** is a modern Python simulator and controller for the **UFACTORY Lite 6** robotic arm. It allows users to simulate movements using Inverse Kinematics (IK), visualize custom end-effectors, and control the physical robot seamlessly from a unified, dark-themed interface.

Built with **PyVista** for 3D rendering and **sv_ttk** for a modern UI experience.

---

## 🚀 Key Features

### 🎮 Simulation & Control
* **Hybrid Workflow:** Works in standalone Simulation Mode or connected to a physical Lite 6 robot.
* **Inverse Kinematics:** Drag sliders to calculate joint angles.
* **Script Playback:** Load and execute Lite 6 Python scripts to automate movements.
* **Real-time Synchronization:** See the robot's status live in the 3D viewer.

### 🛠️ End-Effector Customization
* **Dynamic Loading:** Load custom `.stl` files as end-effectors on the fly.
* **STL Scaling:** Select if files are in millimeters and scales them to meters for accurate sizing.
* **Tip Calculation:** Automatically calculates the length of the tool for accurate path tracing.
* **Presets:** Quick buttons for the standard UFactory Gripper and Vacuum Gripper.

### 👁️ Visualization Tools
* **Trace Path:** Draw lines in 3D space to visualize the robot's path (switchable between Wrist and End-Effector Tip).
* **Ghost Mode:** Make the robot transparent (10% opacity) to view the path clearly, with an option to keep the end-effector fully visible.
* **Custom Colors:** Fully customizable colors for the background, arm, wrist, effector, and trace lines.

---

## 🛠️ Support & Compatibility

LiteSim is developed and tested on modern hardware to ensure the best performance for 3D simulation.

### ✅ Officially Supported
We provide pre-built executables (`.exe` / `.app`) and full support for:
* **Windows 10 / 11** (x64)
* **macOS** (Apple Silicon)

### ⚠️ Apple Intel Users
We **do not** provide pre-built applications for Intel-based Macs.
However, LiteSim is fully compatible with Intel Macs if run directly from the source code. Please set up a Conda or Python environment with the libraries found in `requirements.txt` and run `python main.py` to get started.

---

## ❤️ Credits & Acknowledgment

This project was created by me with extensive assistance from **Gemini 3 Pro** (Google AI). The AI served as a co-pilot for architecture, UI design (sv_ttk integration), and logic implementation.

* **SDK:** [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
* **Visualization:** [PyVista](https://docs.pyvista.org/)
* **IK Solver:** [IKPy](https://github.com/Phylliade/ikpy)
* **Theme:** [Sun Valley TTK](https://github.com/rdbende/Sun-Valley-ttk-theme)

---

## ⚖️ Disclaimer & Rights

**Not affiliated with UFACTORY.**
This simulator is an independent, open-source project developed for educational and experimental purposes. Mostly created for a school project where I had limited time physcially working with the Lite 6, thus having the need for a simulator.

* **3D Models:** All robot models (`.urdf`, `.stl`), meshes, and design specifications of the Lite 6 are the exclusive intellectual property of **[UFACTORY](https://www.ufactory.cc/)**.
* **SDK:** The `xArm-Python-SDK` used to communicate with the physical robot belongs to UFACTORY.

## 📄 License

This project code is open-source. Feel free to use and modify it for your own robotic projects.