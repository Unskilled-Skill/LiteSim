# Qt-based control panel embedding the PyVista visualizer in the same window.
import os
import sys
import threading
import queue
import runpy
import types
import subprocess
import socket
from functools import partial
import ast
import json
from datetime import datetime
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
        QPushButton { background-color: #2b2b2b; border: 1px solid #3a3a3a; border-radius: 4px; padding: 6px 12px; min-height: 28px; color: #e4e7ec; text-align: center; }
        QPushButton:hover { background-color: #333333; border-color: #f08c28; }
        QPushButton:pressed { background-color: #202020; border-color: #f08c28; }
        QPushButton:disabled { color: #6f7378; border-color: #2c2c2c; }
        QLineEdit, QComboBox, QTextEdit, QSpinBox, QDoubleSpinBox { background: #1a1a1a; border: 1px solid #2f2f2f; border-radius: 4px; padding: 4px; selection-background-color: #f08c28; selection-color: #0f0f0f; }
        QComboBox::drop-down { border: 0; width: 18px; }
        QSlider::groove:horizontal { height: 6px; background: #2c2c2c; border-radius: 3px; }
        QSlider::handle:horizontal { width: 14px; background: #f08c28; border: 1px solid #b96a1f; margin: -5px 0; border-radius: 7px; }
        QScrollBar { background: #1a1a1a; }
        QScrollBar::handle { background: #3c3c3c; border-radius: 5px; }
        QScrollBar::handle:hover { background: #505050; }
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
        self.color_presets = {"Default": dict(self.color_vars)}

        self.loop_enabled = False
        self.speed_value = 1.0
        self.scale_mm = True
        self.collision_popup_shown = False
        self.script_history = []
        self.stl_history = []
        self._ip_store_path = os.path.join(config.USER_DATA_DIR, "robot_ip.txt")

        self._build_ui()
        self._load_saved_ip()

        # Create the scene using the Qt interactor as the plotter backend
        self.ik_chain = self.viz.setup_scene(plotter_override=self.viz_widget)
        if not self.ik_chain:
            QtWidgets.QMessageBox.critical(self, "Error", "Could not load URDF model.")
            sys.exit(1)

        # Init API
        self.api = SimXArmAPI(self.ctx, self.ik_chain)
        self.api.speed_multiplier = self.speed_value
        self.api.set_sim_only(True)
        # Stream listener state
        self.stream_host = "127.0.0.1"
        self.stream_port = 7777
        self.stream_status_state = "off"

        # Try to auto-connect to real robot if IP was saved and SDK available
        self._auto_connect_saved_ip()

        self._load_history()
        self._load_stl_history()
        self._refresh_color_presets()

        # Timers
        self.render_timer = QtCore.QTimer(self)
        self.render_timer.timeout.connect(self._update_3d_loop)
        self.render_timer.start(40)

        self.queue_timer = QtCore.QTimer(self)
        self.queue_timer.timeout.connect(self._process_queues)
        self.queue_timer.start(30)

    def _make_collapsible_old(self, title, content_widget, expanded=True):
        """Return a small collapsible container with a toggle header and the given content."""
        container = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(container)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)

        toggle = QtWidgets.QPushButton(("▼ " if expanded else "► ") + title)
        toggle.setCheckable(True)
        toggle.setChecked(expanded)
        toggle.setFlat(True)
        toggle.setFocusPolicy(QtCore.Qt.NoFocus)
        toggle.setStyleSheet("""
            QPushButton {
                border: none;
                background: transparent;
                color: #dfe3e8;
                padding: 2px 0;
                font-weight: 600;
                text-align: left;
            }
            QPushButton:hover,
            QPushButton:pressed,
            QPushButton:checked,
            QPushButton:checked:hover,
            QPushButton:checked:pressed,
            QPushButton:focus {
                border: none;
                background: transparent;
                color: #dfe3e8;
            }
        """)
        vbox.addWidget(toggle)

        frame = QtWidgets.QFrame()
        frame_layout = QtWidgets.QVBoxLayout(frame)
        frame_layout.setContentsMargins(8, 4, 8, 8)
        frame_layout.addWidget(content_widget)
        vbox.addWidget(frame)

        def on_toggle(checked):
            frame.setVisible(checked)
            toggle.setText(("▼ " if checked else "► ") + title)

        toggle.toggled.connect(on_toggle)
        on_toggle(expanded)
        return container


    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QVBoxLayout(central)

        # Compact header actions (top-right)
        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(4, 4, 4, 4)
        header.setSpacing(8)
        # Restart GUI (left-aligned)
        btn_refresh_header = QtWidgets.QPushButton("Restart GUI")
        btn_refresh_header.setToolTip("Restart the GUI (auto relaunches)")
        btn_refresh_header.setFixedHeight(28)
        btn_refresh_header.clicked.connect(self._refresh_gui)
        header.addWidget(btn_refresh_header)

        header.addStretch(1)

        cam_buttons = [
            ("Front", lambda: self.viz.set_camera_view('front', 1.6)),
            ("Left", lambda: self.viz.set_camera_view('side-l', 1.6)),
            ("Right", lambda: self.viz.set_camera_view('side-r', 1.6)),
            ("Rear", lambda: self.viz.set_camera_view('rear', 1.6)),
            ("Top", lambda: self.viz.set_camera_view('top', 1.6)),
            ("Iso", self._reset_view),
        ]
        for text, fn in cam_buttons:
            btn = QtWidgets.QPushButton(text)
            btn.setFixedHeight(28)
            btn.clicked.connect(fn)
            header.addWidget(btn)

        btn_reset_view_global = QtWidgets.QPushButton("Reset View")
        btn_reset_view_global.setFixedHeight(28)
        btn_reset_view_global.clicked.connect(self._reset_view)
        header.addWidget(btn_reset_view_global)
        btn_reset_colors_global = QtWidgets.QPushButton("Reset Colors")
        btn_reset_colors_global.setFixedHeight(28)
        btn_reset_colors_global.clicked.connect(self._reset_all_colors)
        header.addWidget(btn_reset_colors_global)
        main_layout.addLayout(header)

        splitter = QtWidgets.QSplitter()
        splitter.setOrientation(QtCore.Qt.Horizontal)
        splitter.setHandleWidth(8)
        main_layout.addWidget(splitter)

        # Left controls
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        # Extra right margin so vertical scrollbar never overlaps content
        left_layout.setContentsMargins(10, 10, 16, 10)
        left_layout.setSpacing(12)

        # Robot connection (real Lite 6)
        conn_group = QtWidgets.QGroupBox("Robot Connection")
        conn_group.setTitle("")
        conn_layout = QtWidgets.QGridLayout(conn_group)
        conn_layout.setContentsMargins(6, 6, 6, 6)
        conn_layout.setHorizontalSpacing(8)
        conn_layout.setVerticalSpacing(6)

        conn_layout.addWidget(QtWidgets.QLabel("Robot IP:"), 0, 0)
        self.edit_robot_ip = QtWidgets.QLineEdit("192.168.1.xxx")
        self.edit_robot_ip.setPlaceholderText("e.g. 192.168.1.208")
        conn_layout.addWidget(self.edit_robot_ip, 0, 1, 1, 2)

        self.sim_only_chk = QtWidgets.QCheckBox("Sim-only (do not move real arm)")
        self.sim_only_chk.setChecked(True)
        self.sim_only_chk.stateChanged.connect(lambda _: self._toggle_sim_only(self.sim_only_chk.isChecked()))
        conn_layout.addWidget(self.sim_only_chk, 1, 0, 1, 3)

        self.btn_connect_real = QtWidgets.QPushButton("Connect to Robot")
        self.btn_connect_real.clicked.connect(self._toggle_real_connection)
        conn_layout.addWidget(self.btn_connect_real, 2, 0, 1, 2)

        self.btn_disconnect_real = QtWidgets.QPushButton("Disconnect")
        self.btn_disconnect_real.clicked.connect(self._disconnect_real_connection)
        self.btn_disconnect_real.setEnabled(False)
        conn_layout.addWidget(self.btn_disconnect_real, 2, 2)

        self.lbl_conn_status = QtWidgets.QLabel("Status: Disconnected")
        self.lbl_conn_status.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_conn_status.setStyleSheet("color: #d9534f; font-weight: 600;")
        conn_layout.addWidget(self.lbl_conn_status, 3, 0, 1, 3)

        left_layout.addWidget(self._make_collapsible("Robot Connection", conn_group, expanded=False))

        # Script loading/history (top priority)
        script_group = QtWidgets.QGroupBox("Script Loading")
        sg_layout = QtWidgets.QGridLayout(script_group)
        sg_layout.setContentsMargins(6, 6, 6, 6)
        sg_layout.setHorizontalSpacing(8)
        sg_layout.setVerticalSpacing(4)
        self.combo_history = QtWidgets.QComboBox()
        self.combo_history.setMinimumWidth(200)
        self.combo_history.currentIndexChanged.connect(self._on_history_select)
        sg_layout.addWidget(self.combo_history, 0, 0, 1, 2)

        self.btn_browse = QtWidgets.QPushButton("Load")
        self.btn_browse.setMinimumWidth(100)
        self.btn_browse.clicked.connect(self._browse_script)
        sg_layout.addWidget(self.btn_browse, 0, 2)

        self.btn_refresh_scripts = QtWidgets.QPushButton("Refresh")
        self.btn_refresh_scripts.setToolTip("Reload recent + examples")
        self.btn_refresh_scripts.setMinimumWidth(80)
        self.btn_refresh_scripts.clicked.connect(self._load_history)
        sg_layout.addWidget(self.btn_refresh_scripts, 0, 3)

        self.btn_stream_toggle = QtWidgets.QPushButton("Start Live Stream")
        self.btn_stream_toggle.setMinimumWidth(120)
        self.btn_stream_toggle.setCheckable(True)
        self.btn_stream_toggle.setChecked(False)
        self.btn_stream_toggle.clicked.connect(self._toggle_stream_listener)
        sg_layout.addWidget(self.btn_stream_toggle, 1, 0, 1, 2)

        self.lbl_stream_status = QtWidgets.QLabel("Stream: Off")
        self.lbl_stream_status.setStyleSheet("color: #d9534f;")
        self.lbl_stream_status.setAlignment(QtCore.Qt.AlignCenter)
        sg_layout.addWidget(self.lbl_stream_status, 1, 2, 1, 2)

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
            spin.setMinimumWidth(80)
            spin.setRange(*config.JOINT_LIMITS[i])
            spin.setSingleStep(0.5)
            spin.setValue(0.0)
            self.joint_spin.append(spin)
            row.addWidget(spin)

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            min_lim = int(config.JOINT_LIMITS[i][0] * 10)
            max_lim = int(config.JOINT_LIMITS[i][1] * 10)
            slider.setMinimum(min_lim)
            slider.setMaximum(max_lim)
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
        trace_group.setTitle("")
        tr_layout = QtWidgets.QGridLayout(trace_group)
        self.trace_chk = QtWidgets.QCheckBox("Trace Path")
        self.trace_chk.stateChanged.connect(self._toggle_trace)
        tr_layout.addWidget(self.trace_chk, 0, 0, 1, 1)

        tr_layout.addWidget(QtWidgets.QLabel("Source:"), 0, 1, 1, 1)
        self.trace_mode = QtWidgets.QComboBox()
        self.trace_mode.addItems(["Wrist", "Effector Tip"])
        self.trace_mode.currentIndexChanged.connect(self._change_trace_source)
        self.trace_mode.setMinimumWidth(120)
        tr_layout.addWidget(self.trace_mode, 0, 2, 1, 2)

        self.ghost_chk = QtWidgets.QCheckBox("Ghost Mode")
        self.ghost_chk.stateChanged.connect(self._apply_ghost_state)
        tr_layout.addWidget(self.ghost_chk, 1, 0, 1, 1)

        self.ignore_eef_chk = QtWidgets.QCheckBox("Ignore Effector")
        self.ignore_eef_chk.setEnabled(False)
        self.ignore_eef_chk.stateChanged.connect(self._apply_ghost_state)
        tr_layout.addWidget(self.ignore_eef_chk, 1, 1, 1, 1)

        self.collision_alert_chk = QtWidgets.QCheckBox("Collision Alerts")
        self.collision_alert_chk.setChecked(True)
        tr_layout.addWidget(self.collision_alert_chk, 1, 2, 1, 1)
        tr_layout.setColumnStretch(3, 1)
        left_layout.addWidget(self._make_collapsible("Trace & Visibility", trace_group, expanded=False))

        # Color settings (single column, scroll-friendly)
        color_group = QtWidgets.QGroupBox("Color Settings")
        color_group.setTitle("")
        cg_layout = QtWidgets.QGridLayout(color_group)
        cg_layout.setContentsMargins(10, 8, 10, 8)
        cg_layout.setHorizontalSpacing(8)
        cg_layout.setVerticalSpacing(6)
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
            btn_apply.setMinimumWidth(90)
            btn_apply.setMinimumHeight(26)
            btn_apply.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
            btn_apply.clicked.connect(lambda _, k=key: self._apply_color(k))
            cg_layout.addWidget(btn_apply, idx, 3)

            btn_reset = QtWidgets.QPushButton("Reset")
            btn_reset.setMinimumWidth(90)
            btn_reset.setMinimumHeight(26)
            btn_reset.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
            btn_reset.clicked.connect(lambda _, k=key: self._reset_color(k))
            cg_layout.addWidget(btn_reset, idx, 4)

            edit.textChanged.connect(lambda val, k=key: self._update_color_preview(k, val))
            self._update_color_preview(key, self.color_vars[key])

        preset_row = QtWidgets.QHBoxLayout()
        preset_row.setContentsMargins(0, 6, 0, 0)
        preset_row.setSpacing(6)
        preset_row.addWidget(QtWidgets.QLabel("Preset:"))
        self.combo_color_presets = QtWidgets.QComboBox()
        self.combo_color_presets.setMinimumWidth(160)
        self.combo_color_presets.addItem("Default")
        self.combo_color_presets.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        preset_row.addWidget(self.combo_color_presets, 1)
        self.btn_save_preset = QtWidgets.QPushButton("Save Current")
        self.btn_save_preset.setMinimumWidth(110)
        self.btn_save_preset.setMinimumHeight(26)
        self.btn_save_preset.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        preset_row.addWidget(self.btn_save_preset)
        self.btn_load_preset = QtWidgets.QPushButton("Load")
        self.btn_load_preset.setMinimumWidth(80)
        self.btn_load_preset.setMinimumHeight(26)
        self.btn_load_preset.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        preset_row.addWidget(self.btn_load_preset)
        preset_row.addStretch(1)
        cg_layout.addLayout(preset_row, len(labels), 0, 1, 5)

        cg_layout.setColumnStretch(1, 1)
        color_group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        left_layout.addWidget(self._make_collapsible("Color Settings", color_group, expanded=False))

        # End effector config
        ef_group = QtWidgets.QGroupBox("End-Effector Configuration")
        ef_group.setTitle("")
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
        left_layout.addWidget(self._make_collapsible("End-Effector Configuration", ef_group, expanded=False))

        left_layout.addStretch(1)
        left_container = QtWidgets.QWidget()
        left_container.setLayout(left_layout)
        left_container.setMinimumWidth(480)
        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        left_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left_scroll.setStyleSheet("""
            QScrollArea { background: transparent; border: none; }
            QScrollArea > QWidget > QWidget { background: #242424; }
            QScrollBar:vertical {
                background: transparent;
                width: 9px;
                margin: 4px 0 4px 0;
            }
            QScrollBar::handle:vertical {
                background: #454545;
                min-height: 24px;
                border-radius: 5px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }
        """)
        left_scroll.setWidget(left_container)
        left_scroll.setMinimumWidth(480)
        left_scroll.setMaximumWidth(650)
        splitter.addWidget(left_scroll)

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
        self.collision_popup_shown = False
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

    def _reset_all_colors(self):
        defaults = {
            "bg": config.COLOR_BG,
            "arm": config.COLOR_BASE,
            "wrist": config.COLOR_WRIST,
            "eef": config.COLOR_EEF,
            "trace": config.COLOR_PATH,
        }
        for key, val in defaults.items():
            if key in self.color_inputs:
                self.color_inputs[key].setText(val)
                self._apply_color(key)

    def _refresh_gui(self):
        script_path = os.path.abspath(sys.argv[0])
        cmd = [sys.executable, script_path]
        try:
            subprocess.Popen(cmd, cwd=os.path.dirname(script_path))
        except Exception:
            pass
        QtWidgets.QApplication.quit()

    def _preflight_script(self, path):
        """Static check for obvious out-of-limit commands in a script."""
        warnings = []
        try:
            with open(path, "r", encoding="utf-8") as f:
                src = f.read()
            tree = ast.parse(src, filename=path)
        except Exception as e:
            self._append_log(f"[SAFEGUARD] Preflight skipped: {e}")
            return warnings

        joint_limits = getattr(config, "JOINT_LIMITS", [])
        max_joint_speed = getattr(config, "JOINT_SPEED_LIMIT_DEG_S", None)
        max_joint_acc = getattr(config, "JOINT_ACC_LIMIT_DEG_S2", None)
        max_tcp_speed = getattr(config, "TCP_SPEED_LIMIT_MM_S", None)
        max_tcp_acc = getattr(config, "TCP_ACC_LIMIT_MM_S2", None)

        def eval_num(node):
            if isinstance(node, (ast.Num, ast.Constant)) and isinstance(node.value, (int, float)):
                return float(node.value)
            return None

        def eval_list(node):
            if isinstance(node, (ast.List, ast.Tuple)):
                vals = []
                for el in node.elts:
                    v = eval_num(el)
                    if v is None:
                        return None
                    vals.append(v)
                return vals
            return None

        class Checker(ast.NodeVisitor):
            def visit_Call(self, node):
                func_name = ""
                if isinstance(node.func, ast.Attribute):
                    func_name = node.func.attr
                elif isinstance(node.func, ast.Name):
                    func_name = node.func.id
                if func_name in ("set_servo_angle", "set_position"):
                    self.check_call(func_name, node)
                self.generic_visit(node)

            def check_call(self, name, node):
                # positional args
                args = list(node.args)
                kwargs = {kw.arg: kw.value for kw in node.keywords if kw.arg}
                if name == "set_servo_angle":
                    angles_node = kwargs.get("angle") or (args[0] if args else None)
                    angles = eval_list(angles_node) if angles_node is not None else None
                    speed = eval_num(kwargs.get("speed") or (args[1] if len(args) > 1 else None))
                    acc = eval_num(kwargs.get("mvacc") or (args[2] if len(args) > 2 else None))

                    if angles:
                        for idx, val in enumerate(angles):
                            if idx < len(joint_limits):
                                lo, hi = joint_limits[idx]
                                if val < lo or val > hi:
                                    warnings.append(f"Joint {idx+1} target {val} deg outside limits [{lo}, {hi}]")
                    if speed and max_joint_speed and speed > max_joint_speed:
                        warnings.append(f"Joint speed {speed} deg/s exceeds limit {max_joint_speed}")
                    if acc and max_joint_acc and acc > max_joint_acc:
                        warnings.append(f"Joint acc {acc} deg/s^2 exceeds limit {max_joint_acc}")

                if name == "set_position":
                    speed = eval_num(kwargs.get("speed") or (args[6] if len(args) > 6 else None))
                    acc = eval_num(kwargs.get("acc"))
                    if speed and max_tcp_speed and speed > max_tcp_speed:
                        warnings.append(f"TCP speed {speed} mm/s exceeds limit {max_tcp_speed}")
                    if acc and max_tcp_acc and acc > max_tcp_acc:
                        warnings.append(f"TCP acc {acc} mm/s^2 exceeds limit {max_tcp_acc}")

        Checker().visit(tree)
        return warnings
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
        # Preflight analysis for unsafe speeds/limits
        warnings = self._preflight_script(self.current_script_path)
        if warnings:
            msg = QtWidgets.QMessageBox(self)
            msg.setIcon(QtWidgets.QMessageBox.Warning)
            msg.setWindowTitle("Preflight Warnings")
            msg.setText("Potential unsafe commands detected in the script.")
            msg.setInformativeText("Continue anyway?")
            msg.setDetailedText("\n".join(warnings))
            msg.setStandardButtons(QtWidgets.QMessageBox.Cancel | QtWidgets.QMessageBox.Ok)
            msg.setDefaultButton(QtWidgets.QMessageBox.Cancel)
            choice = msg.exec_()
            if choice != QtWidgets.QMessageBox.Ok:
                self._append_log("[SAFEGUARD] Run cancelled due to preflight warnings.")
                return
            else:
                self._append_log("[SAFEGUARD] Proceeding despite warnings.")
        self.collision_popup_shown = False
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

    def _toggle_stream_listener(self):
        if not self.btn_stream_toggle.isChecked():
            # Stop
            self.btn_stream_toggle.setText("Start Live Stream")
            self.btn_stream_toggle.setChecked(False)
            self._set_stream_status("off")
            try:
                self.api.stop_joint_stream_listener()
            except Exception:
                pass
            return
        # Start
        try:
            self.api.start_joint_stream_listener(host=self.stream_host, port=self.stream_port)
            self.btn_stream_toggle.setText("Stop Live Stream")
            self._set_stream_status("listening")
        except Exception as e:
            self.btn_stream_toggle.setChecked(False)
            self._set_stream_status("off")
            self._append_log(f"[STREAM] Failed to start: {e}")

    def _set_stream_status(self, state):
        if state == self.stream_status_state:
            return
        self.stream_status_state = state
        if state == "connected":
            self.lbl_stream_status.setText("Stream: Connected")
            self.lbl_stream_status.setStyleSheet("color: #5cb85c;")  # green
        elif state == "listening":
            self.lbl_stream_status.setText("Stream: Listening")
            self.lbl_stream_status.setStyleSheet("color: #f0ad4e;")  # amber
        else:
            self.lbl_stream_status.setText("Stream: Off")
            self.lbl_stream_status.setStyleSheet("color: #d9534f;")  # red

    # --- Real robot connection ---
    def _toggle_real_connection(self):
        if self.api and self.api.is_connected:
            self._disconnect_real_connection()
            return
        ip = self.edit_robot_ip.text().strip()
        if not ip:
            QtWidgets.QMessageBox.warning(self, "Robot IP", "Please enter the robot IP address.")
            return
        self.btn_connect_real.setEnabled(False)
        self.btn_disconnect_real.setEnabled(False)
        self.edit_robot_ip.setEnabled(False)
        self.btn_connect_real.setText("Connecting...")
        self._set_connection_status("connecting")
        threading.Thread(target=self._connect_real_thread, args=(ip, False), daemon=True).start()

    def _connect_real_thread(self, ip, from_auto=False):
        success, msg = self.api.connect_real_robot(ip)
        QtCore.QTimer.singleShot(0, lambda s=success, m=msg, a=from_auto: self._finish_real_connect(s, m, a))

    def _finish_real_connect(self, success, msg, from_auto=False):
        if success:
            self._save_ip(self.edit_robot_ip.text())
            self._set_connection_status("connected")
            self.btn_connect_real.setText("Disconnect")
            self.btn_disconnect_real.setEnabled(True)
            self.edit_robot_ip.setEnabled(False)
        else:
            if not from_auto:
                QtWidgets.QMessageBox.warning(self, "Connection Failed", f"Could not connect:\n{msg}")
            else:
                self._append_log(f"[REAL] Auto-connect failed: {msg}")
            self._set_connection_status("disconnected")
            self.btn_connect_real.setText("Connect to Robot")
            self.edit_robot_ip.setEnabled(True)
            self.btn_disconnect_real.setEnabled(False)
        self.btn_connect_real.setEnabled(True)

    def _disconnect_real_connection(self):
        try:
            self.api.disconnect_real_robot()
        except Exception:
            pass
        self._save_ip(self.edit_robot_ip.text())
        self._set_connection_status("disconnected")
        self.btn_connect_real.setText("Connect to Robot")
        self.btn_connect_real.setEnabled(True)
        self.edit_robot_ip.setEnabled(True)
        self.btn_disconnect_real.setEnabled(False)

    def _set_connection_status(self, state):
        if not hasattr(self, "lbl_conn_status"):
            return
        if state == "connected":
            text = "Status: Connected"
            color = "#5cb85c"
        elif state == "connecting":
            text = "Status: Connecting..."
            color = "#f0ad4e"
        else:
            text = "Status: Disconnected"
            color = "#d9534f"
        self.lbl_conn_status.setText(text)
        self.lbl_conn_status.setStyleSheet(f"color: {color}; font-weight: 600;")

    def _auto_connect_saved_ip(self):
        """Attempt auto-connect using saved IP if SDK is available."""
        if not getattr(config, "HAS_REAL_SDK", False):
            return
        ip = ""
        try:
            ip = self.edit_robot_ip.text().strip()
        except Exception:
            return
        if not ip:
            return
        # Quick port probe to avoid hanging when nothing is reachable
        try:
            with socket.create_connection((ip, config.ROBOT_SCAN_PORT), timeout=1.0):
                pass
        except Exception:
            self._append_log(f"[REAL] Auto-connect skipped (no robot at {ip})")
            self._set_connection_status("disconnected")
            self.btn_connect_real.setText("Connect to Robot")
            self.btn_connect_real.setEnabled(True)
            self.edit_robot_ip.setEnabled(True)
            self.btn_disconnect_real.setEnabled(False)
            return
        # mimic button state for connect
        self.btn_connect_real.setEnabled(False)
        self.btn_disconnect_real.setEnabled(False)
        self.edit_robot_ip.setEnabled(False)
        self.btn_connect_real.setText("Connecting...")
        self._set_connection_status("connecting")
        threading.Thread(target=self._connect_real_thread, args=(ip, True), daemon=True).start()

    def _load_saved_ip(self):
        try:
            if os.path.exists(self._ip_store_path):
                with open(self._ip_store_path, "r", encoding="utf-8") as f:
                    saved_ip = f.read().strip()
                    if saved_ip:
                        self.edit_robot_ip.setText(saved_ip)
        except Exception:
            pass

    def _save_ip(self, ip_text):
        try:
            with open(self._ip_store_path, "w", encoding="utf-8") as f:
                f.write(ip_text.strip())
        except Exception:
            pass

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

    def _save_color_preset(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Save Color Preset", "Preset name:")
        if not ok or not name.strip():
            return
        name = name.strip()
        self.color_presets[name] = dict(self.color_vars)
        self._refresh_color_presets(selected=name)

    def _load_color_preset(self):
        name = self.combo_color_presets.currentText()
        if name in self.color_presets:
            vals = self.color_presets[name]
            for k, v in vals.items():
                if k in self.color_inputs:
                    self.color_inputs[k].setText(v)
                    self._apply_color(k)

    def _refresh_color_presets(self, selected=None):
        self.combo_color_presets.blockSignals(True)
        self.combo_color_presets.clear()
        for key in self.color_presets.keys():
            self.combo_color_presets.addItem(key)
        if selected and selected in self.color_presets:
            self.combo_color_presets.setCurrentText(selected)
        else:
            self.combo_color_presets.setCurrentText("Default")
        self.combo_color_presets.blockSignals(False)

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
        # Prevent repeated popups; still halt everything
        if self.collision_popup_shown:
            self.ctx.stop_flag = True
            self.ctx.paused = True
            self.running_script = False
            self._toggle_controls(running=False)
            return
        self.collision_popup_shown = True
        # Halt everything but keep current pose so the user can inspect before deciding to reset
        self.ctx.stop_flag = True
        self.ctx.paused = True
        self.running_script = False
        self._toggle_controls(running=False)
        self._append_log("[ALERT] COLLISION DETECTED! Simulation halted.")

        msg = QtWidgets.QMessageBox(self)
        msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.setWindowTitle("Collision Detected")
        msg.setText("The robot hit the floor. Simulation halted.")
        msg.setInformativeText("Choose an action:")
        btn_resume = msg.addButton("Resume", QtWidgets.QMessageBox.AcceptRole)
        btn_reset = msg.addButton("Reset to Home", QtWidgets.QMessageBox.DestructiveRole)
        btn_snapshot = msg.addButton("Save Snapshot", QtWidgets.QMessageBox.ActionRole)
        msg.setDefaultButton(btn_reset)
        msg.exec_()

        clicked = msg.clickedButton()
        if clicked == btn_snapshot:
            path = self._save_collision_snapshot()
            if path:
                self._append_log(f"[ALERT] Snapshot saved: {os.path.basename(path)}")
            # remain halted but clear popup flag so user can choose again if needed
            self.collision_popup_shown = False
            return

        if clicked == btn_reset:
            self._home()
        # Resume: clear pause/stop flags (script remains stopped; user can restart)
        self.ctx.stop_flag = False
        self.ctx.paused = False
        self.collision_popup_shown = False
    def _toggle_sim_only(self, enabled):
        if hasattr(self, "api"):
            try:
                self.api.set_sim_only(enabled)
            except Exception:
                pass

    def _save_collision_snapshot(self):
        """Save current joint state (and minimal context) to a JSON file."""
        try:
            ts = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            fname = f"collision_snapshot_{ts}.json"
            path = os.path.join(config.USER_DATA_DIR, fname)
            payload = {
                "timestamp_utc": ts,
                "joints_deg": list(getattr(self.api, "joints_deg", [])),
                "trace_source": getattr(self.viz, "trace_source", None),
                "loop_enabled": self.loop_enabled,
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            return path
        except Exception as e:
            self._append_log(f"[ALERT] Failed to save snapshot: {e}")
            return None

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

        # Update stream status indicator based on API flags
        if hasattr(self, "api"):
            new_state = "off"
            if getattr(self.api, "stream_running", False):
                new_state = "connected" if getattr(self.api, "stream_connected", False) else "listening"
            self._set_stream_status(new_state)

            # Update real-robot connection indicator
            if hasattr(self, "btn_connect_real") and hasattr(self, "edit_robot_ip"):
                if getattr(self.api, "is_connected", False):
                    self._set_connection_status("connected")
                    self.btn_connect_real.setText("Disconnect")
                    self.btn_disconnect_real.setEnabled(True)
                    self.edit_robot_ip.setEnabled(False)
                else:
                    if not self.btn_connect_real.text().startswith("Connecting"):
                        self._set_connection_status("disconnected")
                        self.btn_connect_real.setText("Connect to Robot")
                        self.btn_disconnect_real.setEnabled(False)
                        self.edit_robot_ip.setEnabled(True)
                # keep sim-only checkbox reflecting API
                if hasattr(self, "sim_only_chk"):
                    self.sim_only_chk.blockSignals(True)
                    self.sim_only_chk.setChecked(getattr(self.api, "sim_only_mode", True))
                    self.sim_only_chk.blockSignals(False)

    # Final override to keep collapsible headers plain (no state styling)
    def _make_collapsible_old2(self, title, content_widget, expanded=True):
        container = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(container)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)

        arrow_open = "▾"
        arrow_closed = "▸"
        toggle = QtWidgets.QPushButton((f"{arrow_open}  " if expanded else f"{arrow_closed}  ") + title)
        toggle.setObjectName("CollapseHeader")
        toggle.setFlat(True)
        toggle.setCheckable(False)
        toggle.setAutoDefault(False)
        toggle.setDefault(False)
        toggle.setFocusPolicy(QtCore.Qt.NoFocus)
        toggle.setStyleSheet("""
            QPushButton#CollapseHeader {
                border: none;
                background: transparent;
                color: #dfe3e8;
                padding: 3px 0;
                font-weight: 600;
                text-align: left;
            }
            QPushButton#CollapseHeader:hover {
                background: #2b2b2b;
                color: #f08c28;
            }
            QPushButton#CollapseHeader:pressed {
                background: #262626;
                color: #f08c28;
            }
            QPushButton#CollapseHeader:focus { outline: none; }
        """)
        vbox.addWidget(toggle)

        frame = QtWidgets.QFrame()
        frame_layout = QtWidgets.QVBoxLayout(frame)
        frame_layout.setContentsMargins(8, 4, 8, 8)
        frame_layout.addWidget(content_widget)
        vbox.addWidget(frame)

        state = {"open": expanded}

        def on_click():
            state["open"] = not state["open"]
            frame.setVisible(state["open"])
            toggle.setText((f"{arrow_open}  " if state["open"] else f"{arrow_closed}  ") + title)

        toggle.clicked.connect(on_click)
        frame.setVisible(expanded)
        return container


    # Override collapsible header to use ASCII arrows and neutral hover
    def _make_collapsible(self, title, content_widget, expanded=True):
        container = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(container)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)

        arrow_open = "v"
        arrow_closed = ">"
        toggle = QtWidgets.QPushButton((f"{arrow_open} " if expanded else f"{arrow_closed} ") + title)
        toggle.setObjectName("CollapseHeader")
        toggle.setFlat(True)
        toggle.setCheckable(False)
        toggle.setAutoDefault(False)
        toggle.setDefault(False)
        toggle.setFocusPolicy(QtCore.Qt.NoFocus)
        toggle.setStyleSheet("""
            QPushButton#CollapseHeader {
                border: none;
                background: transparent;
                color: #dfe3e8;
                padding: 3px 0;
                font-weight: 600;
                text-align: left;
            }
            QPushButton#CollapseHeader:hover {
                background: transparent;
                color: #f08c28;
            }
            QPushButton#CollapseHeader:pressed {
                background: transparent;
                color: #f08c28;
            }
            QPushButton#CollapseHeader:focus { outline: none; }
        """)
        vbox.addWidget(toggle)

        frame = QtWidgets.QFrame()
        frame_layout = QtWidgets.QVBoxLayout(frame)
        frame_layout.setContentsMargins(8, 4, 8, 8)
        frame_layout.addWidget(content_widget)
        vbox.addWidget(frame)

        state = {"open": expanded}

        def on_click():
            state["open"] = not state["open"]
            frame.setVisible(state["open"])
            toggle.setText((f"{arrow_open} " if state["open"] else f"{arrow_closed} ") + title)

        toggle.clicked.connect(on_click)
        frame.setVisible(expanded)
        return container


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_palette(app)
    win = QtControlPanel()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
