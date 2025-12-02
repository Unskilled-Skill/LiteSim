# Qt-based control panel embedding the PyVista visualizer in the same window.
import os
import sys
import threading
import queue
import runpy
import types
from PyQt5 import QtCore, QtWidgets, QtGui
from pyvistaqt import QtInteractor

import config
from robot_api import SimXArmAPI
from visualizer import RobotVisualizer

# --- Qt styling helpers ---
def apply_dark_palette(app: QtWidgets.QApplication):
    """Apply a night-mode palette and widget styling."""
    dark = QtGui.QPalette()
    base = QtGui.QColor("#1f1f1f")
    panel = QtGui.QColor("#242424")
    text = QtGui.QColor("#dfe3e8")
    accent = QtGui.QColor("#f08c28")  # subtle orange like Blender
    dark.setColor(QtGui.QPalette.Window, panel)
    dark.setColor(QtGui.QPalette.WindowText, text)
    dark.setColor(QtGui.QPalette.Base, base)
    dark.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor("#2a2a2a"))
    dark.setColor(QtGui.QPalette.ToolTipBase, panel)
    dark.setColor(QtGui.QPalette.ToolTipText, text)
    dark.setColor(QtGui.QPalette.Text, text)
    dark.setColor(QtGui.QPalette.Button, panel)
    dark.setColor(QtGui.QPalette.ButtonText, text)
    dark.setColor(QtGui.QPalette.BrightText, QtCore.Qt.red)
    dark.setColor(QtGui.QPalette.Highlight, accent)
    dark.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(dark)
    app.setStyleSheet("""
        QWidget { font-family: 'Segoe UI', 'Helvetica Neue', Arial; font-size: 10pt; color: #dfe3e8; }
        QGroupBox { font-weight: 600; border: 1px solid #2f2f2f; border-radius: 6px; margin-top: 8px; padding-top: 10px; background: #1f1f1f; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #f08c28; }
        QFrame, QSplitter { background: #242424; }
        QSplitter::handle { background: #2f2f2f; }
        QPushButton { background-color: #2b2b2b; border: 1px solid #3a3a3a; border-radius: 4px; padding: 6px 10px; color: #e4e7ec; }
        QPushButton:hover { background-color: #333333; border-color: #f08c28; }
        QPushButton:pressed { background-color: #202020; border-color: #f08c28; }
        QPushButton:disabled { color: #6f7378; border-color: #2c2c2c; }
        QLineEdit, QComboBox, QTextEdit, QSpinBox, QDoubleSpinBox { background: #1a1a1a; border: 1px solid #2f2f2f; border-radius: 4px; padding: 4px; selection-background-color: #f08c28; selection-color: #0f0f0f; }
        QComboBox::drop-down { border: 0; width: 18px; }
        QSlider::groove:horizontal { height: 6px; background: #2c2c2c; border-radius: 3px; }
        QSlider::handle:horizontal { width: 14px; background: #f08c28; border: 1px solid #b96a1f; margin: -5px 0; border-radius: 7px; }
        QScrollBar { background: #1a1a1a; }
        QScrollBar::handle { background: #2f2f2f; }
        QLabel { color: #dfe3e8; }
        QCheckBox { spacing: 6px; }
    """)

class AppContext:
    def __init__(self):
        self.log_queue = queue.Queue()
        self.joint_queue = queue.Queue(maxsize=2)
        self.stop_flag = False
        self.paused = False


class QtControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{config.APP_NAME} {config.APP_VERSION} | UFACTORY Lite 6 Simulator | Qt Controls")
        self.resize(1400, 900)

        self.ctx = AppContext()
        self.data_lock = threading.Lock()

        self.viz = RobotVisualizer()
        self.api = None
        self.ik_chain = None
        self.current_script_path = None
        self.running_script = False
        self.was_colliding = False

        self.color_vars = {
            "bg": config.COLOR_BG,
            "arm": config.COLOR_BASE,
            "wrist": config.COLOR_WRIST,
            "eef": config.COLOR_EEF,
            "trace": config.COLOR_PATH,
        }

        self.loop_enabled = False
        self.speed_value = 1.0
        self.scale_mm = True

        self.script_history = []
        self.stl_history = []

        self._build_ui()

        # Create the scene using the Qt interactor as the plotter backend
        self.ik_chain = self.viz.setup_scene(plotter_override=self.viz_widget)
        if not self.ik_chain:
            QtWidgets.QMessageBox.critical(self, "Error", "Could not load URDF model.")
            sys.exit(1)

        # Init API
        self.api = SimXArmAPI(self.ctx, self.ik_chain)
        self.api.speed_multiplier = self.speed_value

        self._load_history()
        self._load_stl_history()

        # Timers
        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self._update_3d_loop)
        self.render_timer.start(40)

        self.queue_timer = QtCore.QTimer(self)
        self.queue_timer.timeout.connect(self._process_queues)
        self.queue_timer.start(30)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)
        splitter = QtWidgets.QSplitter()
        splitter.setOrientation(QtCore.Qt.Horizontal)
        splitter.setHandleWidth(8)
        main_layout.addWidget(splitter)

        # Left controls
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(10, 10, 10, 10)
        left_layout.setSpacing(12)

        # Script loading/history (top priority)
        script_group = QtWidgets.QGroupBox("Script Loading")
        sg_layout = QtWidgets.QHBoxLayout(script_group)
        self.combo_history = QtWidgets.QComboBox()
        self.combo_history.currentIndexChanged.connect(self._on_history_select)
        sg_layout.addWidget(self.combo_history, 1)
        self.btn_browse = QtWidgets.QPushButton("Load .py script...")
        self.btn_browse.clicked.connect(self._browse_script)
        sg_layout.addWidget(self.btn_browse)
        left_layout.addWidget(script_group)

        # Simulation controls
        sim_group = QtWidgets.QGroupBox("Simulation Controls")
        sim_layout = QtWidgets.QGridLayout(sim_group)
        self.btn_run = QtWidgets.QPushButton("Run")
        self.btn_run.clicked.connect(self._run_current_script)
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_stop.clicked.connect(self._stop_script)
        self.btn_restart = QtWidgets.QPushButton("Restart")
        self.btn_restart.clicked.connect(self._restart_script)
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.clicked.connect(self._toggle_pause)
        for b in (self.btn_run, self.btn_pause, self.btn_restart, self.btn_stop):
            b.setMinimumHeight(32)
        sim_layout.addWidget(self.btn_run, 0, 0)
        sim_layout.addWidget(self.btn_pause, 0, 1)
        sim_layout.addWidget(self.btn_restart, 0, 2)
        sim_layout.addWidget(self.btn_stop, 0, 3)

        self.loop_chk = QtWidgets.QCheckBox("Loop Script")
        self.loop_chk.stateChanged.connect(lambda _: self._set_loop(self.loop_chk.isChecked()))
        sim_layout.addWidget(self.loop_chk, 1, 0)

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(50)
        self.speed_slider.setValue(10)
        self.speed_slider.valueChanged.connect(self._on_speed_change)
        self.speed_lbl = QtWidgets.QLabel("1.0x")
        sim_layout.addWidget(QtWidgets.QLabel("Sim Speed:"), 1, 1)
        sim_layout.addWidget(self.speed_slider, 1, 2)
        sim_layout.addWidget(self.speed_lbl, 1, 3)
        left_layout.addWidget(sim_group)

        # Manual joint control
        joints_group = QtWidgets.QGroupBox("Manual Joint Control")
        j_layout = QtWidgets.QVBoxLayout(joints_group)
        self.joint_sliders = []
        self.joint_spin = []
        for i in range(config.JOINT_COUNT):
            row = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"Joint {i+1}")
            row.addWidget(label)

            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(1)
            spin.setRange(*config.JOINT_LIMITS[i])
            spin.setSingleStep(0.5)
            spin.setValue(0.0)
            self.joint_spin.append(spin)
            row.addWidget(spin)

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setMinimum(config.JOINT_LIMITS[i][0] * 10)
            slider.setMaximum(config.JOINT_LIMITS[i][1] * 10)
            slider.setValue(0)
            self.joint_sliders.append(slider)
            row.addWidget(slider, 1)

            slider.valueChanged.connect(lambda val, idx=i: self._on_slider_changed(idx, val))
            spin.valueChanged.connect(lambda val, idx=i: self._on_spin_changed(idx, val))
            j_layout.addLayout(row)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_home = QtWidgets.QPushButton("Reset to Home")
        self.btn_home.clicked.connect(self._home)
        btn_row.addWidget(self.btn_home)
        self.btn_reset_view = QtWidgets.QPushButton("Reset View")
        self.btn_reset_view.clicked.connect(self._reset_view)
        btn_row.addWidget(self.btn_reset_view)
        left_layout.addWidget(joints_group)
        left_layout.addLayout(btn_row)

        # Trace / visibility
        trace_group = QtWidgets.QGroupBox("Trace & Visibility")
        tr_layout = QtWidgets.QHBoxLayout(trace_group)
        self.trace_chk = QtWidgets.QCheckBox("Trace Path")
        self.trace_chk.stateChanged.connect(self._toggle_trace)
        tr_layout.addWidget(self.trace_chk)
        self.trace_mode = QtWidgets.QComboBox()
        self.trace_mode.addItems(["Wrist", "Effector Tip"])
        self.trace_mode.currentIndexChanged.connect(self._change_trace_source)
        tr_layout.addWidget(self.trace_mode)
        self.ghost_chk = QtWidgets.QCheckBox("Ghost Mode")
        self.ghost_chk.stateChanged.connect(self._apply_ghost_state)
        tr_layout.addWidget(self.ghost_chk)
        self.ignore_eef_chk = QtWidgets.QCheckBox("Ignore Effector")
        self.ignore_eef_chk.setEnabled(False)
        self.ignore_eef_chk.stateChanged.connect(self._apply_ghost_state)
        tr_layout.addWidget(self.ignore_eef_chk)
        self.collision_alert_chk = QtWidgets.QCheckBox("Collision Alerts")
        self.collision_alert_chk.setChecked(True)
        tr_layout.addWidget(self.collision_alert_chk)
        left_layout.addWidget(trace_group)

        # Color settings
        color_group = QtWidgets.QGroupBox("Color Settings")
        cg_layout = QtWidgets.QGridLayout(color_group)
        self.color_inputs = {}
        self.color_previews = {}
        labels = [("Background", "bg"), ("Robot Arm", "arm"), ("Wrist", "wrist"), ("End-Effector", "eef"), ("Trace", "trace")]
        for idx, (lbl, key) in enumerate(labels):
            cg_layout.addWidget(QtWidgets.QLabel(lbl + ":"), idx, 0)
            edit = QtWidgets.QLineEdit(self.color_vars[key])
            self.color_inputs[key] = edit
            cg_layout.addWidget(edit, idx, 1)
            preview = QtWidgets.QLabel()
            preview.setFixedSize(22, 22)
            preview.setFrameShape(QtWidgets.QFrame.Box)
            preview.setLineWidth(1)
            self.color_previews[key] = preview
            cg_layout.addWidget(preview, idx, 2)
            btn_apply = QtWidgets.QPushButton("Apply")
            btn_apply.clicked.connect(lambda _, k=key: self._apply_color(k))
            cg_layout.addWidget(btn_apply, idx, 3)
            btn_reset = QtWidgets.QPushButton("Reset")
            btn_reset.clicked.connect(lambda _, k=key: self._reset_color(k))
            cg_layout.addWidget(btn_reset, idx, 4)
            edit.textChanged.connect(lambda val, k=key: self._update_color_preview(k, val))
            self._update_color_preview(key, self.color_vars[key])
        left_layout.addWidget(color_group)

        # End effector config
        ef_group = QtWidgets.QGroupBox("End-Effector Configuration")
        ef_layout = QtWidgets.QVBoxLayout(ef_group)
        self.scale_mm_chk = QtWidgets.QCheckBox("Custom end-effector is in millimeters?")
        self.scale_mm_chk.setChecked(True)
        self.scale_mm_chk.stateChanged.connect(lambda _: self._set_scale_mm(self.scale_mm_chk.isChecked()))
        ef_layout.addWidget(self.scale_mm_chk)
        stl_row = QtWidgets.QHBoxLayout()
        self.combo_stls = QtWidgets.QComboBox()
        self.combo_stls.currentIndexChanged.connect(self._on_stl_history_select)
        stl_row.addWidget(self.combo_stls, 1)
        self.btn_load_stl = QtWidgets.QPushButton("Load Custom STL...")
        self.btn_load_stl.clicked.connect(self._load_custom_gripper)
        stl_row.addWidget(self.btn_load_stl)
        ef_layout.addLayout(stl_row)
        preset_row = QtWidgets.QHBoxLayout()
        self.btn_preset_std = QtWidgets.QPushButton("Default Gripper")
        self.btn_preset_std.clicked.connect(lambda: self._load_specific_gripper("gripper_lite.stl"))
        preset_row.addWidget(self.btn_preset_std)
        self.btn_preset_vac = QtWidgets.QPushButton("Vacuum Gripper")
        self.btn_preset_vac.clicked.connect(lambda: self._load_specific_gripper("vacuum_gripper_lite.stl"))
        preset_row.addWidget(self.btn_preset_vac)
        self.btn_preset_remove = QtWidgets.QPushButton("Remove")
        self.btn_preset_remove.clicked.connect(self._remove_gripper)
        preset_row.addWidget(self.btn_preset_remove)
        ef_layout.addLayout(preset_row)
        left_layout.addWidget(ef_group)

        # Camera controls
        cam_group = QtWidgets.QGroupBox("Camera Controls")
        cam_layout = QtWidgets.QGridLayout(cam_group)
        cam_buttons = [
            ("Front", lambda: self.viz.set_camera_view('front', 1.6)),
            ("Left", lambda: self.viz.set_camera_view('side-l', 1.6)),
            ("Right", lambda: self.viz.set_camera_view('side-r', 1.6)),
            ("Rear", lambda: self.viz.set_camera_view('rear', 1.6)),
            ("Top", lambda: self.viz.set_camera_view('top', 1.6)),
            ("Iso", self._reset_view),
        ]
        for col, (text, fn) in enumerate(cam_buttons):
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(fn)
            cam_layout.addWidget(btn, 0, col)
        left_layout.addWidget(cam_group)

        left_layout.addStretch(1)
        left_container = QtWidgets.QWidget()
        left_container.setLayout(left_layout)
        left_container.setMinimumWidth(360)
        left_container.setMaximumWidth(420)
        splitter.addWidget(left_container)

        # Right side: viewer and log
        right_split = QtWidgets.QSplitter()
        right_split.setOrientation(QtCore.Qt.Vertical)

        # Viewer
        self.viz_widget = QtInteractor(None)
        viewer_container = QtWidgets.QWidget()
        v_layout = QtWidgets.QVBoxLayout(viewer_container)
        v_layout.setContentsMargins(0, 0, 0, 0)
        v_layout.addWidget(self.viz_widget)
        right_split.addWidget(viewer_container)

        # Log
        log_container = QtWidgets.QWidget()
        log_layout = QtWidgets.QVBoxLayout(log_container)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        right_split.addWidget(log_container)

        splitter.addWidget(right_split)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 1000])
        right_split.setSizes([750, 200])

    # ---------- History helpers ----------
    def _load_history(self):
        self.script_history = []
        if os.path.exists(config.HISTORY_FILE):
            try:
                with open(config.HISTORY_FILE, "r") as f:
                    for line in f:
                        path = line.strip()
                        if path and os.path.exists(path) and path not in self.script_history:
                            self.script_history.append(path)
            except Exception:
                pass
        if os.path.exists(config.EXAMPLES_DIR):
            try:
                for filename in os.listdir(config.EXAMPLES_DIR):
                    if filename.endswith(".py"):
                        full_path = os.path.join(config.EXAMPLES_DIR, filename)
                        if full_path not in self.script_history:
                            self.script_history.append(full_path)
            except Exception:
                pass
        display = []
        for p in self.script_history:
            name = os.path.basename(p)
            if config.EXAMPLES_DIR in p:
                name = f"[Example] {name}"
            display.append(name)
        self.combo_history.clear()
        self.combo_history.addItems(display)
        if display:
            self.combo_history.setCurrentIndex(0)
            self.current_script_path = self.script_history[0]
            self.btn_run.setEnabled(True)
        else:
            self.btn_run.setEnabled(False)

    def _add_to_history(self, path):
        if path in self.script_history:
            self.script_history.remove(path)
        self.script_history.insert(0, path)
        display = []
        for p in self.script_history:
            name = os.path.basename(p)
            if config.EXAMPLES_DIR in p:
                name = f"[Example] {name}"
            display.append(name)
        self.combo_history.clear()
        self.combo_history.addItems(display)
        if display:
            self.combo_history.setCurrentIndex(0)
        try:
            user_scripts = [p for p in self.script_history if config.EXAMPLES_DIR not in p]
            with open(config.HISTORY_FILE, "w") as f:
                for p in user_scripts[:10]:
                    f.write(p + "\n")
        except Exception:
            pass

    def _on_history_select(self, idx):
        if idx >= 0 and idx < len(self.script_history):
            self.current_script_path = self.script_history[idx]
            self.btn_run.setEnabled(True)

    def _load_stl_history(self):
        self.stl_history = []
        if os.path.exists(config.STL_HISTORY_FILE):
            try:
                with open(config.STL_HISTORY_FILE, "r") as f:
                    for line in f:
                        path = line.strip()
                        if path and os.path.exists(path) and path not in self.stl_history:
                            self.stl_history.append(path)
            except Exception:
                pass
        self.combo_stls.clear()
        display = [os.path.basename(p) for p in self.stl_history]
        if display:
            self.combo_stls.addItem("Select recent STL...")
            self.combo_stls.addItems(display)
        else:
            self.combo_stls.addItem("No STL history")
            self.combo_stls.setEnabled(False)

    def _add_to_stl_history(self, path):
        if path in self.stl_history:
            self.stl_history.remove(path)
        self.stl_history.insert(0, path)
        self.stl_history = self.stl_history[:10]
        try:
            with open(config.STL_HISTORY_FILE, "w") as f:
                for p in self.stl_history:
                    f.write(p + "\n")
        except Exception:
            pass
        self.combo_stls.setEnabled(True)
        self._load_stl_history()

    def _append_log(self, msg):
        self.log_text.append(msg)
        self.log_text.moveCursor(QtGui.QTextCursor.End)

    def _on_slider_changed(self, idx, val):
        # slider value is *10
        deg = val / 10.0
        self.joint_spin[idx].blockSignals(True)
        self.joint_spin[idx].setValue(deg)
        self.joint_spin[idx].blockSignals(False)
        self._apply_joint_changes()

    def _on_spin_changed(self, idx, val):
        self.joint_sliders[idx].blockSignals(True)
        self.joint_sliders[idx].setValue(int(val * 10))
        self.joint_sliders[idx].blockSignals(False)
        self._apply_joint_changes()

    def _apply_joint_changes(self):
        with self.data_lock:
            current = [spin.value() for spin in self.joint_spin]
            self.api.joints_deg = current
            self.viz.update_joints(current)

    def _home(self):
        self.ctx.log_queue.put("[GUI] Going home...")
        with self.data_lock:
            zeros = [0.0] * config.JOINT_COUNT
            for spin, slider in zip(self.joint_spin, self.joint_sliders):
                spin.blockSignals(True)
                slider.blockSignals(True)
                spin.setValue(0.0)
                slider.setValue(0)
                spin.blockSignals(False)
                slider.blockSignals(False)
            self.api.joints_deg = zeros
            self.viz.update_joints(zeros)
        if self.api.real_arm:
            self.api.set_servo_angle([0] * 6, speed=30, wait=False)

    def _reset_view(self):
        if self.viz:
            self.viz.reset_camera_view()

    def _browse_script(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select script", os.getcwd(), "Python Files (*.py)")
        if path:
            self._add_to_history(path)
            self.current_script_path = path
            self._append_log(f"[GUI] Selected: {os.path.basename(path)}")
            self.btn_run.setEnabled(True)

    def _run_current_script(self):
        if not self.current_script_path or self.running_script:
            return
        self.running_script = True
        self._toggle_controls(running=True)
        self.ctx.stop_flag = False
        self.ctx.paused = False
        threading.Thread(target=self._run_script_thread, args=(self.current_script_path,), daemon=True).start()

    def _restart_script(self):
        self._stop_script()
        QtCore.QTimer.singleShot(300, self._run_current_script)

    def _stop_script(self):
        self.ctx.stop_flag = True
        self.ctx.paused = False
        self._append_log("[UI] Stop signal...")

    def _toggle_pause(self):
        if self.ctx.paused:
            self.ctx.paused = False
            self.btn_pause.setText("Pause")
            self._append_log("[UI] Resumed")
        else:
            self.ctx.paused = True
            self.btn_pause.setText("Resume")
            self._append_log("[UI] Paused")

    def _set_loop(self, enabled):
        self.loop_enabled = enabled

    def _on_speed_change(self, val):
        self.speed_value = val / 10.0
        self.speed_lbl.setText(f"{self.speed_value:.1f}x")
        if self.api:
            self.api.speed_multiplier = self.speed_value

    def _toggle_trace(self, *_):
        enabled = self.trace_chk.isChecked()
        self.viz.set_trace_enable(enabled)

    def _change_trace_source(self, *_):
        mode = self.trace_mode.currentText().lower()
        self.viz.trace_source = mode
        self.viz.clear_trace()

    def _apply_ghost_state(self, *_):
        is_ghost = self.ghost_chk.isChecked()
        ignore_eef = self.ignore_eef_chk.isChecked()
        self.ignore_eef_chk.setEnabled(is_ghost)
        self.viz.set_ghost_mode(is_ghost, ignore_eef)

    def _apply_color(self, key):
        raw = self.color_inputs[key].text().strip()
        val = self._normalize_color(raw)
        if not val:
            self._update_color_preview(key, raw)
            return
        if self.viz.set_color(key if key != "bg" else "bg", val):
            self.color_vars[key] = val
            self._update_color_preview(key, val)

    def _reset_color(self, key):
        default = {
            "bg": config.COLOR_BG,
            "arm": config.COLOR_BASE,
            "wrist": config.COLOR_WRIST,
            "eef": config.COLOR_EEF,
            "trace": config.COLOR_PATH,
        }[key]
        self.color_inputs[key].setText(default)
        self._apply_color(key)

    def _update_color_preview(self, key, val):
        if key not in self.color_previews:
            return
        norm = self._normalize_color(val)
        if not norm:
            self.color_previews[key].setStyleSheet("background: #1d1f23; border: 1px solid #a33;")
        else:
            self.color_previews[key].setStyleSheet(f"background: {norm}; border: 1px solid #3c3f43;")

    def _normalize_color(self, val):
        if not val:
            return None
        s = val.strip()
        if not s:
            return None
        hex_chars = set("0123456789abcdefABCDEF")
        if not s.startswith("#") and all(c in hex_chars for c in s) and len(s) in (3, 6):
            s = "#" + s
        color = QtGui.QColor(s)
        if not color.isValid():
            return None
        return color.name()

    def _set_scale_mm(self, enabled):
        self.scale_mm = enabled

    def _load_custom_gripper(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Choose an STL file", os.getcwd(), "STL files (*.stl)")
        if path:
            do_scale = self.scale_mm
            success = self.viz.set_custom_gripper(path, scale_to_meters=do_scale)
            if success:
                self._append_log(f"[GUI] Loaded: {os.path.basename(path)}")
                self._add_to_stl_history(path)
                self._force_trace_mode("Effector Tip")
            else:
                QtWidgets.QMessageBox.critical(self, "Error", "Could not load the STL file.")

    def _load_specific_gripper(self, filename):
        full_path = self.viz.get_mesh_path(filename)
        if not full_path:
            QtWidgets.QMessageBox.critical(self, "Not found", f"Cannot find file:\n{filename}")
            return
        success = self.viz.set_custom_gripper(full_path, scale_to_meters=False)
        if success:
            self._append_log(f"[GUI] Loaded preset: {filename}")
            self._force_trace_mode("Effector Tip")
        else:
            QtWidgets.QMessageBox.critical(self, "Error", f"Cannot load {filename}.")

    def _remove_gripper(self):
        success = self.viz.remove_gripper()
        if success:
            self._append_log("[GUI] End-effector removed from model")
            self._force_trace_mode("Wrist")

    def _force_trace_mode(self, mode_text):
        self.trace_mode.setCurrentText(mode_text)
        self.viz.trace_source = mode_text.lower()
        self.viz.clear_trace()

    def _on_stl_history_select(self, idx):
        if idx <= 0:
            return
        offset = 1 if self.combo_stls.itemText(0) == "Select recent STL..." else 0
        real_idx = idx - offset
        if real_idx >= 0 and real_idx < len(self.stl_history):
            path = self.stl_history[real_idx]
            do_scale = self.scale_mm
            success = self.viz.set_custom_gripper(path, scale_to_meters=do_scale)
            if success:
                self._append_log(f"[GUI] Loaded from history: {os.path.basename(path)}")
                self._force_trace_mode("Effector Tip")

    def _toggle_controls(self, running):
        stl_enabled = (len(self.stl_history) > 0) and (not running)
        self.btn_run.setEnabled(not running and self.current_script_path is not None)
        self.btn_browse.setEnabled(not running)
        self.combo_history.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        self.btn_restart.setEnabled(running)
        self.btn_pause.setEnabled(running)
        self.speed_slider.setEnabled(not running)
        self.loop_chk.setEnabled(not running)
        self.btn_load_stl.setEnabled(not running)
        self.combo_stls.setEnabled(stl_enabled)
        self.btn_preset_std.setEnabled(not running)
        self.btn_preset_vac.setEnabled(not running)
        self.btn_preset_remove.setEnabled(not running)

    def _run_script_thread(self, path):
        # Mirror Tk behavior for the xarm stub
        if 'xarm' in sys.modules:
            del sys.modules['xarm']
        if 'xarm.wrapper' in sys.modules:
            del sys.modules['xarm.wrapper']
        xarm_mod = types.ModuleType('xarm')
        wrap_mod = types.ModuleType('xarm.wrapper')

        live_api_instance = self.api

        def API_Factory(ip, **kwargs):
            return live_api_instance

        wrap_mod.XArmAPI = API_Factory
        xarm_mod.wrapper = wrap_mod
        sys.modules['xarm'] = xarm_mod
        sys.modules['xarm.wrapper'] = wrap_mod

        self.ctx.log_queue.put(f"--- Start: {os.path.basename(path)} ---")
        try:
            runpy.run_path(path, run_name="__main__")
        except SystemExit as e:
            self.ctx.log_queue.put(f"--- {e} ---")
        except Exception as e:
            self.ctx.log_queue.put(f"Error: {e}")
        finally:
            self.ctx.log_queue.put("--- Done ---")
            self.ctx.log_queue.put("__SCRIPT_DONE__")

    def _on_script_finished(self):
        self.running_script = False
        self._toggle_controls(running=False)
        self.ctx.paused = False
        self.btn_pause.setText("Pause")
        if self.ctx.stop_flag:
            self.ctx.log_queue.put("[LOOP] Stopped by user.")
            self._home()
            return
        if self.loop_enabled:
            self.ctx.log_queue.put("[LOOP] Looping...")
            QtCore.QTimer.singleShot(0, self._run_current_script)

    def _update_3d_loop(self):
        if not self.viz or not self.viz.plotter:
            return
        try:
            is_collision = self.viz.render_frame()
            if is_collision:
                self.was_colliding = True
                if self.collision_alert_chk.isChecked():
                    self._handle_collision()
                    return
            else:
                if self.was_colliding:
                    for key, val in self.color_vars.items():
                        if key in ["arm", "wrist", "eef"]:
                            self.viz.set_color(key, val)
                    self.was_colliding = False
        except Exception:
            pass

    def _handle_collision(self):
        self.ctx.stop_flag = True
        self.ctx.paused = True
        self._append_log("[ALERT] COLLISION DETECTED! Robot hit the floor.")
        QtWidgets.QMessageBox.critical(self, "COLLISION DETECTED", "The robot hit the floor. Simulation paused.\nClick OK to reset to Home.")
        self._stop_script()
        self._home()
        for key, val in self.color_vars.items():
            if key in ["arm", "wrist", "eef"]:
                self.viz.set_color(key, val)
        QtCore.QTimer.singleShot(200, self._resume_from_crash)

    def _resume_from_crash(self):
        self.ctx.paused = False
        self._update_3d_loop()

    def _process_queues(self):
        # Logs
        while not self.ctx.log_queue.empty():
            try:
                msg = self.ctx.log_queue.get_nowait()
            except queue.Empty:
                break
            if msg == "__SCRIPT_DONE__":
                self._on_script_finished()
                continue
            self._append_log(str(msg))

        # Joint updates from API
        latest_joints = None
        while not self.ctx.joint_queue.empty():
            try:
                latest_joints = self.ctx.joint_queue.get_nowait()
            except queue.Empty:
                break

        if latest_joints:
            with self.data_lock:
                self.viz.update_joints(latest_joints)
            for idx, val in enumerate(latest_joints):
                if idx < len(self.joint_spin):
                    self.joint_spin[idx].blockSignals(True)
                    self.joint_spin[idx].setValue(val)
                    self.joint_spin[idx].blockSignals(False)
                    self.joint_sliders[idx].blockSignals(True)
                    self.joint_sliders[idx].setValue(int(val * 10))
                    self.joint_sliders[idx].blockSignals(False)


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_palette(app)
    win = QtControlPanel()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
