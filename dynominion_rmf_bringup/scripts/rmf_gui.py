#!/usr/bin/env python3
import sys
import os
import json
import uuid
import time
import subprocess
import signal
import requests
import threading

# Guard rclpy import so the script can fail gracefully if the environment isn't sourced
RCLPY_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rmf_fleet_msgs.msg import FleetState
    from rmf_task_msgs.msg import TaskSummary, ApiRequest, ApiResponse
    from rmf_traffic_msgs.msg import Heartbeat
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool
    RCLPY_AVAILABLE = True
except ImportError:
    pass

# PyQt5 Imports
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QGridLayout, QGroupBox, QPushButton, QLabel, QComboBox,
        QSpinBox, QTextEdit, QTabWidget, QProgressBar, QMessageBox, QFrame
    )
    from PyQt5.QtCore import QThread, pyqtSignal, QProcess, Qt, QTimer
    from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor
except ImportError:
    print("Error: PyQt5 is not installed! Install it using 'pip install PyQt5' or check your python environment.")
    sys.exit(1)


# ─── COLOR SYSTEM (HSL & Hex harmonies) ──────────────────────────────────────
BG_COLOR = "#0f0f16"         # Deep obsidian backdrop
PANEL_COLOR = "#181825"      # Sleek panel background
CARD_COLOR = "#1e1e2e"       # Card element background
BORDER_COLOR = "#313244"     # Dark charcoal borders
TEXT_MAIN = "#cdd6f4"        # Crisp off-white text
TEXT_MUTED = "#a6adc8"       # Secondary text color

ACCENT_BLUE = "#89b4fa"      # Electric blue highlights
ACCENT_LAVENDER = "#cba6f7"  # Glowing lavender highlights
ACCENT_CYAN = "#89dceb"      # Cyan active highlights

COLOR_GREEN = "#a6e3a1"      # Emerald green status
COLOR_RED = "#f38ba8"        # Coral red status / errors
COLOR_YELLOW = "#f9e2af"     # Golden amber warning / bidding
COLOR_GREY = "#45475a"       # Disconnected status


# ─── ROS 2 BACKGROUND WORKER THREAD ──────────────────────────────────────────
class ROS2Worker(QThread):
    # PyQt signals to communicate with the GUI Thread safely
    fleet_updated = pyqtSignal(dict)
    task_summary_received = pyqtSignal(dict)
    api_response_received = pyqtSignal(str, bool, dict) # request_id, success, data
    infra_heartbeat = pyqtSignal(str, bool)             # component, online_status
    event_logged = pyqtSignal(str, str)                 # level, message

    def __init__(self):
        super().__init__()
        self._running = True
        self.node = None
        self.pub_api_request = None
        self.last_clock_time = 0
        self.clock_active = False

    def run(self):
        if not RCLPY_AVAILABLE:
            self.event_logged.emit("error", "ROS2 rclpy is not available in current environment!")
            return

        rclpy.init()
        self.node = Node("rmf_gui_controller")
        
        # Configure simulation time
        self.node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Transitive QoS for RMF API
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ── Subscribers & Publishers ─────────────────────────────────────────
        # Subscriptions
        self.node.create_subscription(FleetState, "/fleet_states", self._fleet_state_cb, transient_qos)
        self.node.create_subscription(TaskSummary, "/task_summaries", self._task_summary_cb, transient_qos)
        self.node.create_subscription(ApiResponse, "/task_api_responses", self._api_response_cb, transient_qos)
        self.node.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.node.create_subscription(Heartbeat, "/rmf_traffic/heartbeat", self._rmf_heartbeat_cb, 10)

        # Publisher
        self.pub_api_request = self.node.create_publisher(ApiRequest, "/task_api_requests", transient_qos)

        self.event_logged.emit("info", "Background ROS 2 node 'rmf_gui_controller' started successfully.")

        # Main spinning loop
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Cleanup
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self._running = False
        self.wait()

    # ── Callback Handlers ────────────────────────────────────────────────────
    def _clock_cb(self, msg):
        self.last_clock_time = msg.clock.sec
        if not self.clock_active:
            self.clock_active = True
            self.infra_heartbeat.emit("gazebo", True)
            self.event_logged.emit("ok", "Gazebo Clock discovered on topic /clock")

    def _rmf_heartbeat_cb(self, _msg):
        self.infra_heartbeat.emit("rmf_core", True)

    def _fleet_state_cb(self, msg):
        # We detected the fleet state which means RMF / Fleet Adapter is live!
        self.infra_heartbeat.emit("fleet_adapter", True)
        
        # Helper to get closest waypoint from nav graph coordinates
        def get_closest_waypoint(x, y):
            waypoints_map = {
                "charger_main": (1.487, -5.618),
                "lobby_waypoint": (1.487, -9.519),
                "hub_center": (7.890, -9.519),
                "room_entry": (7.920, -15.951),
                "room_inside": (3.334, -19.406),
                "corridor_bottom": (1.636, -16.666),
                "bottom_zone": (2.113, -11.663),
                "corridor_north": (7.711, -5.558)
            }
            closest_name = None
            min_dist = float("inf")
            for name, (wx, wy) in waypoints_map.items():
                dist = ((x - wx) ** 2 + (y - wy) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    closest_name = name
            if min_dist < 0.8:  # within 80 cm tolerance to cover slight offsets
                return closest_name
            return f"({x:.1f}, {y:.1f})"

        # Build dictionary to emit back to UI
        robots_data = []
        for robot in msg.robots:
            # Map RMF Mode integers to beautiful names
            mode_map = {
                0: "IDLE", 1: "CHARGING", 2: "NAVIGATING", 3: "PAUSED",
                4: "WAITING", 5: "EMERGENCY", 6: "RETURNING", 7: "DOCKING"
            }
            mode_name = mode_map.get(robot.mode.mode, f"MODE_{robot.mode.mode}")
            
            # Extract position and compute nearest waypoint
            wp_str = "—"
            if hasattr(robot, "location") and robot.location:
                wp_str = get_closest_waypoint(robot.location.x, robot.location.y)

            robots_data.append({
                "name": robot.name,
                "battery": round(robot.battery_percent, 1),
                "mode": mode_name,
                "waypoint": wp_str,
                "task_id": robot.task_id if robot.task_id else "—"
            })
        
        self.fleet_updated.emit({
            "fleet_name": msg.name,
            "robots": robots_data
        })

    def _task_summary_cb(self, msg):
        states_map = {
            0: "QUEUED", 1: "SELECTED", 2: "DISPATCHED",
            3: "ACTIVE", 4: "FAILED", 5: "COMPLETED",
            6: "PENDING", 7: "CANCELLED"
        }
        state_name = states_map.get(msg.state, f"STATE_{msg.state}")
        
        self.task_summary_received.emit({
            "task_id": msg.task_id,
            "state": state_name,
            "robot": msg.robot_name if msg.robot_name else "—",
            "time": time.strftime("%H:%M:%S")
        })
        self.event_logged.emit("ok", f"Task Update -> ID: {msg.task_id} is {state_name} on Robot: {msg.robot_name}")

    def _api_response_cb(self, msg):
        try:
            res_data = json.loads(msg.json_msg)
            success = "success" in msg.json_msg.lower() or res_data.get("success", False) or "error" not in res_data
            self.api_response_received.emit(msg.request_id, success, res_data)
            
            level = "ok" if success else "warn"
            self.event_logged.emit(level, f"Task API Response received for ID: {msg.request_id}")
        except Exception as e:
            self.event_logged.emit("error", f"Failed to parse API Response JSON: {str(e)}")

    # ── Task Sender ──────────────────────────────────────────────────────────
    def publish_task(self, category, destination, params=None):
        if not self.pub_api_request:
            self.event_logged.emit("error", "Cannot publish task: ROS 2 node is not initialized yet!")
            return False

        # Build start time based on current ROS clock (standard unix_millis format)
        now_sec = self.last_clock_time if self.last_clock_time > 0 else int(time.time())
        start_time_ms = now_sec * 1000 + 2000 # Add a tiny 2-second offset to avoid schedule lag

        req_id = "gui_" + str(uuid.uuid4())[:8]

        # 1. Handle GoTo Task Description
        if category == "goto":
            payload = {
                "type": "dispatch_task_request",
                "request": {
                    "category": "compose",
                    "description": {
                        "category": "go_to_place",
                        "phases": [
                            {
                                "activity": {
                                    "category": "go_to_place",
                                    "description": {
                                        "one_of": [{"waypoint": destination}]
                                    }
                                }
                            }
                        ]
                    },
                    "unix_millis_earliest_start_time": start_time_ms
                }
            }
        
        # 2. Handle Custom JSON Task
        elif category == "json":
            try:
                # Custom JSON text must be parsed and patched with request_id and earliest_start_time
                payload = json.loads(params.get("raw_json", "{}"))
                if "request" in payload and "unix_millis_earliest_start_time" in payload["request"]:
                    payload["request"]["unix_millis_earliest_start_time"] = start_time_ms
            except Exception as e:
                self.event_logged.emit("error", f"Invalid raw JSON task format: {str(e)}")
                return False
        
        else:
            self.event_logged.emit("error", f"Task type {category} not supported directly via API yet. Using CLI.")
            return False

        # Publish the structured message
        msg = ApiRequest()
        msg.request_id = req_id
        msg.json_msg = json.dumps(payload)
        self.pub_api_request.publish(msg)

        self.event_logged.emit("info", f"Published direct ApiRequest ID: {req_id} for {category} task")
        return True


# ─── MAIN APP APPLICATION WINDOW ─────────────────────────────────────────────
class RMFDashboardWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dynominion RMF Operations Control Dashboard")
        self.resize(1200, 800)

        # Backend Processes
        self.sim_process = QProcess(self)
        self.rmf_process = QProcess(self)
        
        # Senders
        self.sim_process.readyReadStandardOutput.connect(self._handle_sim_stdout)
        self.sim_process.readyReadStandardError.connect(self._handle_sim_stderr)
        self.rmf_process.readyReadStandardOutput.connect(self._handle_rmf_stdout)
        self.rmf_process.readyReadStandardError.connect(self._handle_rmf_stderr)

        self.sim_process.finished.connect(self._sim_finished)
        self.rmf_process.finished.connect(self._rmf_finished)

        # Health statuses
        self.gazebo_online = False
        self.rmf_core_online = False
        self.fleet_manager_online = False
        self.fleet_adapter_online = False

        # Keep tracking tasks
        self.submitted_tasks = {}

        # ROS 2 Worker Setup
        self.ros_worker = None
        if RCLPY_AVAILABLE:
            self.ros_worker = ROS2Worker()
            self.ros_worker.fleet_updated.connect(self._update_fleet_ui)
            self.ros_worker.task_summary_received.connect(self._update_tasks_ui)
            self.ros_worker.api_response_received.connect(self._handle_api_response)
            self.ros_worker.infra_heartbeat.connect(self._handle_infra_heartbeat)
            self.ros_worker.event_logged.connect(self.log_event)
            self.ros_worker.start()
        else:
            QTimer.singleShot(500, self._show_ros2_warning)

        # Build GUI Layout
        self._init_ui()

        # Timer to regularly check fleet manager Rest API (HTTP check)
        self.rest_timer = QTimer(self)
        self.rest_timer.timeout.connect(self._check_fleet_manager_http)
        self.rest_timer.start(2500) # every 2.5s

        # Apply Premium Style Sheet
        self.setStyleSheet(self._get_stylesheet())

    def _show_ros2_warning(self):
        QMessageBox.critical(
            self, 
            "ROS 2 Environment Error",
            "Unable to import 'rclpy' or RMF messaging packages!\n\n"
            "Please make sure to source your ROS 2 Jazzy installation and "
            "workspace setup files before launching this script:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source install/setup.bash"
        )
        self.log_event("error", "ROS 2 dependencies not found. Sourcing is required.")

    # ─── UI CONSTRUCTION ─────────────────────────────────────────────────────
    def _init_ui(self):
        # Main central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 15, 15, 15)
        main_layout.setSpacing(10)

        # Shared Static Waypoint list
        waypoints = [
            "charger_main", "lobby_waypoint", "hub_center", "room_entry", 
            "room_inside", "corridor_bottom", "bottom_zone", "corridor_north"
        ]

        # ── 1. HEADER SECTION ────────────────────────────────────────────────
        header_frame = QFrame()
        header_frame.setObjectName("HeaderFrame")
        header_layout = QHBoxLayout(header_frame)
        header_layout.setContentsMargins(15, 10, 15, 10)

        title_lbl = QLabel("DYNOMINION RMF MONITOR & DASHBOARD")
        title_lbl.setFont(QFont("Inter", 16, QFont.Bold))
        title_lbl.setStyleSheet(f"color: {ACCENT_LAVENDER}; letter-spacing: 1px;")

        time_lbl = QLabel()
        time_lbl.setFont(QFont("JetBrains Mono", 11))
        time_lbl.setStyleSheet(f"color: {TEXT_MUTED};")
        time_timer = QTimer(self)
        time_timer.timeout.connect(lambda: time_lbl.setText(time.strftime("📅 %Y-%m-%d  🕒 %H:%M:%S")))
        time_timer.start(1000)

        header_layout.addWidget(title_lbl)
        header_layout.addStretch()
        header_layout.addWidget(time_lbl)
        main_layout.addWidget(header_frame)

        # ── 2. MIDDLE SPLIT SECTION ──────────────────────────────────────────
        middle_layout = QHBoxLayout()
        middle_layout.setSpacing(15)

        # LEFT COLUMN (Infrastructure control + Status)
        left_col = QVBoxLayout()
        left_col.setSpacing(12)

        # Group A: Process Control
        control_group = QGroupBox("🖥️ INFRASTRUCTURE LAUNCH CONTROLS")
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(10)

        # Simulation Launcher
        sim_ctrl = QHBoxLayout()
        self.sim_status_led = QLabel("●")
        self.sim_status_led.setStyleSheet(f"color: {COLOR_GREY}; font-size: 14pt;")
        self.sim_status_lbl = QLabel("Simulation Fleet: STOPPED")
        self.sim_status_lbl.setFont(QFont("Inter", 10, QFont.Bold))
        self.btn_start_sim = QPushButton("Launch Simulation")
        self.btn_start_sim.clicked.connect(self._launch_sim)
        self.btn_stop_sim = QPushButton("Stop Sim")
        self.btn_stop_sim.clicked.connect(self._stop_sim)
        self.btn_stop_sim.setEnabled(False)

        sim_ctrl.addWidget(self.sim_status_led)
        sim_ctrl.addWidget(self.sim_status_lbl)
        sim_ctrl.addStretch()
        sim_ctrl.addWidget(self.btn_start_sim)
        sim_ctrl.addWidget(self.btn_stop_sim)
        control_layout.addLayout(sim_ctrl)

        # Separator line
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setFrameShadow(QFrame.Sunken)
        sep.setStyleSheet("background-color: #313244;")
        control_layout.addWidget(sep)

        # RMF Core Launcher
        rmf_ctrl = QHBoxLayout()
        self.rmf_status_led = QLabel("●")
        self.rmf_status_led.setStyleSheet(f"color: {COLOR_GREY}; font-size: 14pt;")
        self.rmf_status_lbl = QLabel("RMF Core System: STOPPED")
        self.rmf_status_lbl.setFont(QFont("Inter", 10, QFont.Bold))
        self.btn_start_rmf = QPushButton("Launch RMF Core")
        self.btn_start_rmf.clicked.connect(self._launch_rmf)
        self.btn_stop_rmf = QPushButton("Stop RMF")
        self.btn_stop_rmf.clicked.connect(self._stop_rmf)
        self.btn_stop_rmf.setEnabled(False)

        rmf_ctrl.addWidget(self.rmf_status_led)
        rmf_ctrl.addWidget(self.rmf_status_lbl)
        rmf_ctrl.addStretch()
        rmf_ctrl.addWidget(self.btn_start_rmf)
        rmf_ctrl.addWidget(self.btn_stop_rmf)
        control_layout.addLayout(rmf_ctrl)

        left_col.addWidget(control_group)

        # Group B: Infrastructure Diagnostics LEDs
        diag_group = QGroupBox("🖥️ SYSTEM DIAGNOSTIC STATUS")
        diag_layout = QGridLayout(diag_group)
        diag_layout.setVerticalSpacing(8)

        self.led_gazebo = QLabel("❌ Offline")
        self.led_gazebo.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
        self.led_rmf = QLabel("❌ Offline")
        self.led_rmf.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
        self.led_fleet_mgr = QLabel("❌ Offline")
        self.led_fleet_mgr.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
        self.led_fleet_adapter = QLabel("❌ Offline")
        self.led_fleet_adapter.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")

        diag_layout.addWidget(QLabel("Gazebo Simulation Clock:"), 0, 0)
        diag_layout.addWidget(self.led_gazebo, 0, 1)
        diag_layout.addWidget(QLabel("RMF Core Schedule Services:"), 1, 0)
        diag_layout.addWidget(self.led_rmf, 1, 1)
        diag_layout.addWidget(QLabel("Fleet Manager HTTP REST API:"), 2, 0)
        diag_layout.addWidget(self.led_fleet_mgr, 2, 1)
        diag_layout.addWidget(QLabel("RMF Fleet Adapter Active:"), 3, 0)
        diag_layout.addWidget(self.led_fleet_adapter, 3, 1)

        left_col.addWidget(diag_group)
        left_col.addStretch()
        middle_layout.addLayout(left_col, 4)

        # RIGHT COLUMN (Fleet state + Tasks)
        right_col = QVBoxLayout()
        right_col.setSpacing(12)

        # Group C: Robot Fleet Status Cards
        fleet_group = QGroupBox("🤖 ACTIVE FLEET OVERVIEW")
        self.fleet_layout = QHBoxLayout(fleet_group)
        self.fleet_layout.setSpacing(10)
        
        # Prepopulate with 5 disconnected cards
        self.robot_cards = {}
        for i in range(1, 6):
            name = f"dynominion{i}"
            card = self._create_robot_card(name)
            self.fleet_layout.addWidget(card)
            self.robot_cards[name] = card

        right_col.addWidget(fleet_group, 2)

        # Group D: Task Dispatcher (Tabbed)
        task_group = QGroupBox("📋 LIVE TASK DISPATCHER")
        task_layout = QVBoxLayout(task_group)
        
        self.task_tabs = QTabWidget()
        
        # 1. Quick Assignment Preset tab
        preset_tab = QWidget()
        preset_layout = QGridLayout(preset_tab)
        preset_layout.setContentsMargins(10, 10, 10, 10)
        preset_layout.setSpacing(10)

        preset_layout.addWidget(QLabel("Click a preset below to instantly dispatch predefined tasks:"), 0, 0, 1, 2)

        btn_lobby = QPushButton("🚀 GoTo Lobby")
        btn_lobby.clicked.connect(lambda: self._dispatch_quick_preset("goto_lobby"))
        btn_lobby.setObjectName("PresetButton")
        preset_layout.addWidget(btn_lobby, 1, 0)

        btn_hub = QPushButton("🚀 GoTo Hub")
        btn_hub.clicked.connect(lambda: self._dispatch_quick_preset("goto_hub"))
        btn_hub.setObjectName("PresetButton")
        preset_layout.addWidget(btn_hub, 1, 1)

        btn_room = QPushButton("🚀 GoTo Room")
        btn_room.clicked.connect(lambda: self._dispatch_quick_preset("goto_room"))
        btn_room.setObjectName("PresetButton")
        preset_layout.addWidget(btn_room, 2, 0)

        btn_bottom = QPushButton("🚀 GoTo Bottom")
        btn_bottom.clicked.connect(lambda: self._dispatch_quick_preset("goto_bottom"))
        btn_bottom.setObjectName("PresetButton")
        preset_layout.addWidget(btn_bottom, 2, 1)

        btn_patrol = QPushButton("🔄 Patrol: Lobby <-> Hub (x2)")
        btn_patrol.clicked.connect(lambda: self._dispatch_quick_preset("patrol"))
        btn_patrol.setObjectName("PresetButtonAccent")
        preset_layout.addWidget(btn_patrol, 3, 0)

        btn_loop = QPushButton("🔄 Loop: Charger <-> North (x2)")
        btn_loop.clicked.connect(lambda: self._dispatch_quick_preset("loop"))
        btn_loop.setObjectName("PresetButtonAccent")
        preset_layout.addWidget(btn_loop, 3, 1)

        self.task_tabs.addTab(preset_tab, "⚡ Modular Presets")

        # 2. GoTo Form tab
        goto_tab = QWidget()
        goto_form = QVBoxLayout(goto_tab)
        
        goto_form.addWidget(QLabel("Select Waypoint Destination:"))
        self.combo_goto_place = QComboBox()
        self.combo_goto_place.addItems(waypoints)
        goto_form.addWidget(self.combo_goto_place)
        
        btn_submit_goto = QPushButton("Send GoTo Place Task")
        btn_submit_goto.clicked.connect(self._submit_goto_form)
        goto_form.addWidget(btn_submit_goto)
        goto_form.addStretch()
        self.task_tabs.addTab(goto_tab, "📍 GoTo Place")

        # 3. Patrol / Loop Custom Form tab
        patrol_tab = QWidget()
        patrol_form = QVBoxLayout(patrol_tab)
        
        patrol_form.addWidget(QLabel("Start Waypoint:"))
        self.combo_patrol_start = QComboBox()
        self.combo_patrol_start.addItems(waypoints)
        patrol_form.addWidget(self.combo_patrol_start)
        
        patrol_form.addWidget(QLabel("Finish Waypoint:"))
        self.combo_patrol_end = QComboBox()
        self.combo_patrol_end.addItems(waypoints)
        self.combo_patrol_end.setCurrentIndex(2) # Default difference
        patrol_form.addWidget(self.combo_patrol_end)

        loop_count_layout = QHBoxLayout()
        loop_count_layout.addWidget(QLabel("Number of Loops:"))
        self.spin_loops = QSpinBox()
        self.spin_loops.setRange(1, 10)
        self.spin_loops.setValue(2)
        loop_count_layout.addWidget(self.spin_loops)
        patrol_form.addLayout(loop_count_layout)

        btn_submit_patrol = QPushButton("Submit Patrol / Loop Task")
        btn_submit_patrol.clicked.connect(self._submit_patrol_form)
        patrol_form.addWidget(btn_submit_patrol)
        patrol_form.addStretch()
        self.task_tabs.addTab(patrol_tab, "🔄 Custom Loop")

        # 4. Custom JSON Raw tab
        json_tab = QWidget()
        json_layout = QVBoxLayout(json_tab)
        
        json_layout.addWidget(QLabel("Raw JSON payload (ApiRequest Format):"))
        self.raw_json_edit = QTextEdit()
        # Prepopulate with elegant sample boilerplate
        sample_json = {
            "type": "dispatch_task_request",
            "request": {
                "category": "compose",
                "description": {
                    "category": "go_to_place",
                    "phases": [
                        {
                            "activity": {
                                "category": "go_to_place",
                                "description": {
                                    "one_of": [{"waypoint": "lobby_waypoint"}]
                                }
                            }
                        }
                    ]
                },
                "unix_millis_earliest_start_time": 0
            }
        }
        self.raw_json_edit.setPlainText(json.dumps(sample_json, indent=2))
        self.raw_json_edit.setFont(QFont("JetBrains Mono", 9))
        json_layout.addWidget(self.raw_json_edit)

        btn_submit_json = QPushButton("Publish Raw JSON API Msg")
        btn_submit_json.clicked.connect(self._submit_raw_json)
        json_layout.addWidget(btn_submit_json)
        
        self.task_tabs.addTab(json_tab, "🔧 Custom JSON")

        task_layout.addWidget(self.task_tabs)
        right_col.addWidget(task_group, 3)

        middle_layout.addLayout(right_col, 6)
        main_layout.addLayout(middle_layout)

        # ── 3. BOTTOM PANE (Event Tracer & Process Terminal Logs) ────────────
        bottom_frame = QFrame()
        bottom_frame.setObjectName("ConsolePane")
        bottom_layout = QVBoxLayout(bottom_frame)
        bottom_layout.setContentsMargins(10, 10, 10, 10)

        ctrl_bar = QHBoxLayout()
        ctrl_bar.addWidget(QLabel("📝 PIPELINE EVENTS & CONSOLE LOGS"))
        ctrl_bar.addStretch()
        
        self.btn_clear_logs = QPushButton("Clear Logs")
        self.btn_clear_logs.clicked.connect(self._clear_logs)
        self.btn_clear_logs.setFixedWidth(100)
        ctrl_bar.addWidget(self.btn_clear_logs)
        
        bottom_layout.addLayout(ctrl_bar)

        self.console_tabs = QTabWidget()

        # Tab 1: Live event log tracer
        self.event_log_view = QTextEdit()
        self.event_log_view.setReadOnly(True)
        self.event_log_view.setFont(QFont("JetBrains Mono", 9))
        self.console_tabs.addTab(self.event_log_view, "📊 Pipeline Event Tracer")

        # Tab 2: Sim launch logs
        self.sim_log_view = QTextEdit()
        self.sim_log_view.setReadOnly(True)
        self.sim_log_view.setFont(QFont("JetBrains Mono", 9))
        self.console_tabs.addTab(self.sim_log_view, "🖥️ Sim Fleet Launch Console")

        # Tab 3: RMF launch logs
        self.rmf_log_view = QTextEdit()
        self.rmf_log_view.setReadOnly(True)
        self.rmf_log_view.setFont(QFont("JetBrains Mono", 9))
        self.console_tabs.addTab(self.rmf_log_view, "⚙️ RMF Core Launch Console")

        bottom_layout.addWidget(self.console_tabs)
        main_layout.addWidget(bottom_frame)

        # Log system setup info
        self.log_event("info", "Dynominion Control Panel Initialized.")
        if not RCLPY_AVAILABLE:
            self.log_event("error", "ROS 2 is offline! Sourcing has failed to make 'rclpy' packages visible.")
        else:
            self.log_event("ok", "ROS 2 worker initialized and seeking network discovery...")

    def _create_robot_card(self, robot_name):
        card = QFrame()
        card.setObjectName("RobotCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        # Name label
        name_lbl = QLabel(robot_name)
        name_lbl.setObjectName("RobotCardTitle")
        name_lbl.setFont(QFont("Inter", 11, QFont.Bold))
        name_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(name_lbl)

        # Status label
        status_lbl = QLabel("DISCONNECTED")
        status_lbl.setObjectName("RobotStatusText")
        status_lbl.setFont(QFont("Inter", 9, QFont.Bold))
        status_lbl.setStyleSheet(f"color: {COLOR_GREY};")
        status_lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(status_lbl)

        # Battery gauge
        bat_layout = QHBoxLayout()
        bat_lbl = QLabel("🔋 Batt:")
        bat_lbl.setFont(QFont("Inter", 9))
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)
        self.battery_bar.setTextVisible(True)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #45475a; }")
        
        bat_layout.addWidget(bat_lbl)
        bat_layout.addWidget(self.battery_bar)
        layout.addLayout(bat_layout)

        # Waypoint label
        wp_lbl = QLabel("📍 Waypoint: —")
        wp_lbl.setObjectName("RobotCardWp")
        wp_lbl.setFont(QFont("Inter", 9))
        layout.addWidget(wp_lbl)

        # Task ID label
        tid_lbl = QLabel("📋 Task: —")
        tid_lbl.setObjectName("RobotCardTid")
        tid_lbl.setFont(QFont("Inter", 8))
        tid_lbl.setStyleSheet(f"color: {TEXT_MUTED};")
        layout.addWidget(tid_lbl)

        # Keep child widget references in card object dictionary
        card.name_lbl = name_lbl
        card.status_lbl = status_lbl
        card.battery_bar = self.battery_bar
        card.wp_lbl = wp_lbl
        card.tid_lbl = tid_lbl

        return card

    # ─── LOGGING & EVENT CAPTURING ───────────────────────────────────────────
    def log_event(self, level, message):
        timestamp = time.strftime("[%H:%M:%S]")
        
        # Color codes
        color_map = {
            "ok": COLOR_GREEN,
            "warn": COLOR_YELLOW,
            "error": COLOR_RED,
            "info": ACCENT_BLUE
        }
        color = color_map.get(level.lower(), TEXT_MAIN)
        header = f"<font color='{ACCENT_LAVENDER}'>{timestamp}</font> "
        
        prefix_map = {
            "ok": f"<font color='{COLOR_GREEN}'>[OK]  </font>",
            "warn": f"<font color='{COLOR_YELLOW}'>[WARN]</font>",
            "error": f"<font color='{COLOR_RED}'>[FAIL]</font>",
            "info": f"<font color='{ACCENT_BLUE}'>[INFO]</font>"
        }
        prefix = prefix_map.get(level.lower(), "")
        
        formatted_message = f"{header} {prefix} <font color='{TEXT_MAIN}'>{message}</font>"
        
        self.event_log_view.append(formatted_message)
        self.event_log_view.moveCursor(QTextCursor.End)

    def _append_console_output(self, text_edit, bytes_data):
        try:
            # Decode cleanly, replacing bad characters
            text = bytes_data.decode("utf-8", errors="replace")
            text_edit.append(text.strip("\r\n"))
            text_edit.moveCursor(QTextCursor.End)
        except Exception:
            pass

    def _clear_logs(self):
        current_tab = self.console_tabs.currentIndex()
        if current_tab == 0:
            self.event_log_view.clear()
        elif current_tab == 1:
            self.sim_log_view.clear()
        elif current_tab == 2:
            self.rmf_log_view.clear()

    # ─── QPROCESS LAUNCH MANAGERS ────────────────────────────────────────────
    def _launch_sim(self):
        if self.sim_process.state() != QProcess.NotRunning:
            return

        self.log_event("info", "Starting Simulation Launch (sim_fleet.launch.py)...")
        self.btn_start_sim.setEnabled(False)
        self.btn_stop_sim.setEnabled(True)

        # ROS 2 Command: sourcing and launching
        cmd = "source /opt/ros/jazzy/setup.bash && source /home/jazzy/dynominion_rmf/src/install/setup.bash && ros2 launch dynominion_rmf_bringup sim_fleet.launch.py"
        self.sim_process.start("bash", ["-c", cmd])
        
        self.sim_status_led.setStyleSheet(f"color: {COLOR_YELLOW}; font-size: 14pt;")
        self.sim_status_lbl.setText("Simulation Fleet: BOOTING")

    def _stop_sim(self):
        self.log_event("warn", "Shutting down Simulation Fleet. Sending SIGINT (KeyboardInterrupt)...")
        self.btn_stop_sim.setEnabled(False)
        
        # Robust ROS 2 graceful termination via SIGINT pkill sweeps
        # Sourcing setup is not strictly necessary for simple pkill commands but we ensure exact match
        subprocess.run(["pkill", "-2", "-f", "sim_fleet.launch.py"])
        
        # Secondary fallback shutdown for deep processes
        self.sim_process.terminate()
        
        # Timer check to make sure it closes cleanly
        QTimer.singleShot(2500, self._cleanup_sim_process)

    def _cleanup_sim_process(self):
        if self.sim_process.state() != QProcess.NotRunning:
            self.log_event("error", "Sim process unresponsive! Force killing leftovers...")
            self.sim_process.kill()
            # Safety sweep
            subprocess.run(["pkill", "-9", "-f", "sim_fleet.launch.py"])
            subprocess.run(["pkill", "-9", "-f", "multi_robot_gazebo.launch.py"])
            subprocess.run(["pkill", "-9", "-f", "gzserver"])
            subprocess.run(["pkill", "-9", "-f", "gzclient"])

    def _sim_finished(self, exit_code, exit_status):
        self.log_event("info", f"Sim Fleet launch process terminated (Exit code: {exit_code})")
        self.btn_start_sim.setEnabled(True)
        self.btn_stop_sim.setEnabled(False)
        self.sim_status_led.setStyleSheet(f"color: {COLOR_GREY}; font-size: 14pt;")
        self.sim_status_lbl.setText("Simulation Fleet: STOPPED")
        
        # Reset diag led
        self.gazebo_online = False
        self.led_gazebo.setText("❌ Offline")
        self.led_gazebo.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
        self._reset_all_robot_cards()

    def _handle_sim_stdout(self):
        self._append_console_output(self.sim_log_view, self.sim_process.readAllStandardOutput())

    def _handle_sim_stderr(self):
        self._append_console_output(self.sim_log_view, self.sim_process.readAllStandardError())

    # RMF Core launcher
    def _launch_rmf(self):
        if self.rmf_process.state() != QProcess.NotRunning:
            return

        self.log_event("info", "Starting RMF Core Systems Launch (rmf_core.launch.py)...")
        self.btn_start_rmf.setEnabled(False)
        self.btn_stop_rmf.setEnabled(True)

        cmd = "source /opt/ros/jazzy/setup.bash && source /home/jazzy/dynominion_rmf/src/install/setup.bash && ros2 launch dynominion_rmf_bringup rmf_core.launch.py"
        self.rmf_process.start("bash", ["-c", cmd])
        
        self.rmf_status_led.setStyleSheet(f"color: {COLOR_YELLOW}; font-size: 14pt;")
        self.rmf_status_lbl.setText("RMF Core System: BOOTING")

    def _stop_rmf(self):
        self.log_event("warn", "Shutting down RMF Core Systems. Sending SIGINT...")
        self.btn_stop_rmf.setEnabled(False)
        
        # Robust pkill sweeps
        subprocess.run(["pkill", "-2", "-f", "rmf_core.launch.py"])
        self.rmf_process.terminate()

        QTimer.singleShot(2500, self._cleanup_rmf_process)

    def _cleanup_rmf_process(self):
        if self.rmf_process.state() != QProcess.NotRunning:
            self.log_event("error", "RMF core process unresponsive! Force killing...")
            self.rmf_process.kill()
            subprocess.run(["pkill", "-9", "-f", "rmf_core.launch.py"])
            subprocess.run(["pkill", "-9", "-f", "fleet_adapter_node"])
            subprocess.run(["pkill", "-9", "-f", "fleet_manager_node"])

    def _rmf_finished(self, exit_code, exit_status):
        self.log_event("info", f"RMF Core launch process terminated (Exit code: {exit_code})")
        self.btn_start_rmf.setEnabled(True)
        self.btn_stop_rmf.setEnabled(False)
        self.rmf_status_led.setStyleSheet(f"color: {COLOR_GREY}; font-size: 14pt;")
        self.rmf_status_lbl.setText("RMF Core System: STOPPED")

        # Reset indicators
        self.rmf_core_online = False
        self.fleet_adapter_online = False
        self.led_rmf.setText("❌ Offline")
        self.led_rmf.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
        self.led_fleet_adapter.setText("❌ Offline")
        self.led_fleet_adapter.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")

    def _handle_rmf_stdout(self):
        self._append_console_output(self.rmf_log_view, self.rmf_process.readAllStandardOutput())

    def _handle_rmf_stderr(self):
        self._append_console_output(self.rmf_log_view, self.rmf_process.readAllStandardError())

    # ─── SYSTEM HEARTBEAT & DIAGNOSTIC SLOTS ─────────────────────────────────
    def _handle_infra_heartbeat(self, component, online):
        if component == "gazebo" and not self.gazebo_online:
            self.gazebo_online = True
            self.led_gazebo.setText("✅ Online (Clock active)")
            self.led_gazebo.setStyleSheet(f"color: {COLOR_GREEN}; font-weight: bold;")
            self.sim_status_led.setStyleSheet(f"color: {COLOR_GREEN}; font-size: 14pt;")
            self.sim_status_lbl.setText("Simulation Fleet: ACTIVE")
            
        elif component == "rmf_core" and not self.rmf_core_online:
            self.rmf_core_online = True
            self.led_rmf.setText("✅ Online (Traffic active)")
            self.led_rmf.setStyleSheet(f"color: {COLOR_GREEN}; font-weight: bold;")
            self.rmf_status_led.setStyleSheet(f"color: {COLOR_GREEN}; font-size: 14pt;")
            self.rmf_status_lbl.setText("RMF Core System: ACTIVE")
            
        elif component == "fleet_adapter" and not self.fleet_adapter_online:
            self.fleet_adapter_online = True
            self.led_fleet_adapter.setText("✅ Online (Fleet registered)")
            self.led_fleet_adapter.setStyleSheet(f"color: {COLOR_GREEN}; font-weight: bold;")

    def _check_fleet_manager_http(self):
        # Asynchronous thread-safe HTTP poll on background thread to keep UI smooth
        def http_probe():
            try:
                robots_rest_data = {}
                is_online = False
                for r_name in ["dynominion1", "dynominion2", "dynominion3", "dynominion4", "dynominion5"]:
                    r = requests.get(f"http://127.0.0.1:8080/v1/robots/{r_name}/state", timeout=1.0)
                    if r.status_code == 200:
                        is_online = True
                        data = r.json()
                        robots_rest_data[r_name] = {
                            "mode": data.get("state", "IDLE"),
                            "task_id": data.get("task_id", "—"),
                            "pose": data.get("pose", [0.0, 0.0, 0.0])
                        }
            except Exception:
                is_online = False
                robots_rest_data = {}
            
            # Use single shot timer to post back to UI thread
            QTimer.singleShot(0, lambda: self._update_fleet_manager_status(is_online, robots_rest_data))

        threading.Thread(target=http_probe, daemon=True).start()

    def _update_fleet_manager_status(self, is_online, robots_rest_data=None):
        if is_online and not self.fleet_manager_online:
            self.fleet_manager_online = True
            self.led_fleet_mgr.setText("✅ Online (HTTP Port 8080)")
            self.led_fleet_mgr.setStyleSheet(f"color: {COLOR_GREEN}; font-weight: bold;")
            self.log_event("ok", "Fleet Manager HTTP REST API online at http://127.0.0.1:8080")
        elif not is_online and self.fleet_manager_online:
            self.fleet_manager_online = False
            self.led_fleet_mgr.setText("❌ Offline")
            self.led_fleet_mgr.setStyleSheet(f"color: {COLOR_RED}; font-weight: bold;")
            self.log_event("error", "Fleet Manager HTTP REST API is offline!")

        # If online, we can update the robot status directly from the REST API!
        if is_online and robots_rest_data:
            # We also compute the nearest waypoint names just like in _fleet_state_cb!
            def get_closest_waypoint(x, y):
                waypoints_map = {
                    "charger_main": (1.487, -5.618),
                    "lobby_waypoint": (1.487, -9.519),
                    "hub_center": (7.890, -9.519),
                    "room_entry": (7.920, -15.951),
                    "room_inside": (3.334, -19.406),
                    "corridor_bottom": (1.636, -16.666),
                    "bottom_zone": (2.113, -11.663),
                    "corridor_north": (7.711, -5.558)
                }
                closest_name = None
                min_dist = float("inf")
                for name, (wx, wy) in waypoints_map.items():
                    dist = ((x - wx) ** 2 + (y - wy) ** 2) ** 0.5
                    if dist < min_dist:
                        min_dist = dist
                        closest_name = name
                if min_dist < 0.8:
                    return closest_name
                return f"({x:.1f}, {y:.1f})"

            for r_name, r_data in robots_rest_data.items():
                if r_name in self.robot_cards:
                    card = self.robot_cards[r_name]
                    mode = r_data["mode"]
                    card.status_lbl.setText(mode)
                    
                    # Color based on mode
                    mode_color = COLOR_GREEN if mode == "IDLE" else (COLOR_YELLOW if mode == "NAVIGATING" else ACCENT_CYAN)
                    card.status_lbl.setStyleSheet(f"color: {mode_color}; font-weight: bold;")
                    
                    # Update waypoint
                    px, py = r_data["pose"][0], r_data["pose"][1]
                    card.wp_lbl.setText(f"📍 Waypoint: {get_closest_waypoint(px, py)}")
                    
                    # Update task id
                    task_id = r_data["task_id"]
                    card.tid_lbl.setText(f"📋 Task: {task_id if task_id else '—'}")

    # ─── FLEET / CARDS STATE RENDERING ───────────────────────────────────────
    def _update_fleet_ui(self, fleet_data):
        robots = fleet_data.get("robots", [])
        for robot in robots:
            name = robot.get("name")
            if name in self.robot_cards:
                card = self.robot_cards[name]
                
                # Battery Percentage is always updated
                battery = robot.get("battery", 0.0)
                card.battery_bar.setValue(int(battery))
                if battery > 50:
                    card.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #a6e3a1; }")
                elif battery > 20:
                    card.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #f9e2af; }")
                else:
                    card.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #f38ba8; }")
                
                # If Fleet Manager is online via REST API, we prefer the REST API's high-fidelity status
                # to prevent flickering and correctly display NAVIGATING/IDLE states!
                if self.fleet_manager_online:
                    continue

                card.status_lbl.setText(robot.get("mode", "UNKNOWN"))
                
                # Dynamic mode colors
                mode_color = COLOR_GREEN if robot.get("mode") == "IDLE" else (COLOR_YELLOW if robot.get("mode") == "NAVIGATING" else ACCENT_CYAN)
                card.status_lbl.setStyleSheet(f"color: {mode_color}; font-weight: bold;")
                
                # Location and Task ID
                card.wp_lbl.setText(f"📍 Waypoint: {robot.get('waypoint')}")
                card.tid_lbl.setText(f"📋 Task: {robot.get('task_id')}")

    def _reset_all_robot_cards(self):
        for name, card in self.robot_cards.items():
            card.status_lbl.setText("DISCONNECTED")
            card.status_lbl.setStyleSheet(f"color: {COLOR_GREY};")
            card.battery_bar.setValue(0)
            card.battery_bar.setStyleSheet("QProgressBar::chunk { background-color: #45475a; }")
            card.wp_lbl.setText("📍 Waypoint: —")
            card.tid_lbl.setText("📋 Task: —")

    def _update_tasks_ui(self, summary):
        # We can add this task state update to the Event Log
        pass

    def _handle_api_response(self, request_id, success, data):
        # Triggered when RMF accepts or rejects a task we published
        pass

    # ─── TASK DISPATCH INVOCATIONS ───────────────────────────────────────────
    def _dispatch_quick_preset(self, preset_name):
        self.log_event("info", f"One-Click preset selected: {preset_name}")
        
        # Valid presets map exactly to dispatch_sample_tasks.sh
        if preset_name == "goto_lobby":
            self._send_cli_task("dispatch_go_to_place", ["-p", "lobby_waypoint", "-F", "dynominion_fleet"])
        elif preset_name == "goto_hub":
            self._send_cli_task("dispatch_go_to_place", ["-p", "hub_center", "-F", "dynominion_fleet"])
        elif preset_name == "goto_room":
            self._send_cli_task("dispatch_go_to_place", ["-p", "room_entry", "-F", "dynominion_fleet"])
        elif preset_name == "goto_bottom":
            self._send_cli_task("dispatch_go_to_place", ["-p", "bottom_zone", "-F", "dynominion_fleet"])
        elif preset_name == "patrol":
            self._send_cli_task("dispatch_patrol", ["-p", "lobby_waypoint", "hub_center", "-n", "2", "-F", "dynominion_fleet"])
        elif preset_name == "loop":
            self._send_cli_task("dispatch_loop", ["-s", "charger_main", "-f", "corridor_north", "-n", "2", "-F", "dynominion_fleet"])

    def _submit_goto_form(self):
        destination = self.combo_goto_place.currentText()
        if not destination:
            return
        
        # Use direct API Topic Dispatching! It is extremely fast
        if self.ros_worker and self.ros_worker.pub_api_request:
            self.ros_worker.publish_task("goto", destination)
        else:
            # Fallback to CLI in case ROS context is not yet active
            self._send_cli_task("dispatch_go_to_place", ["-p", destination, "-F", "dynominion_fleet"])

    def _submit_patrol_form(self):
        start = self.combo_patrol_start.currentText()
        end = self.combo_patrol_end.currentText()
        loops = str(self.spin_loops.value())

        # Patrol dispatches via CLI sequence tool
        self._send_cli_task("dispatch_loop", ["-s", start, "-f", end, "-n", loops, "-F", "dynominion_fleet"])

    def _submit_raw_json(self):
        raw_text = self.raw_json_edit.toPlainText()
        try:
            # Verify valid JSON
            json.loads(raw_text)
        except Exception as e:
            QMessageBox.warning(self, "Invalid JSON", f"Unable to parse JSON payload:\n{str(e)}")
            return

        if self.ros_worker and self.ros_worker.pub_api_request:
            self.ros_worker.publish_task("json", None, {"raw_json": raw_text})
        else:
            QMessageBox.warning(self, "ROS 2 Offline", "The background ROS 2 node is not initialized yet. Raw JSON submission requires ROS 2 connectivity!")

    def _send_cli_task(self, executable, args_list):
        # We run the task sender script asynchronously to prevent UI freeze, capturing logs
        def run_cli():
            self.log_event("info", f"Executing RMF task tool: ros2 run rmf_demos_tasks {executable} {' '.join(args_list)}")
            
            cmd = f"source /opt/ros/jazzy/setup.bash && source /home/jazzy/dynominion_rmf/src/install/setup.bash && ros2 run rmf_demos_tasks {executable} {' '.join(args_list)}"
            
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)
            stdout, stderr = p.communicate()
            
            # Settle outputs to events
            out_str = stdout.decode("utf-8").strip()
            err_str = stderr.decode("utf-8").strip()
            
            if out_str:
                for line in out_str.split("\n"):
                    if "submitted" in line.lower() or "success" in line.lower() or "response" in line.lower():
                        QTimer.singleShot(0, lambda line=line: self.log_event("ok", f"Dispatcher response: {line}"))
            if err_str:
                for line in err_str.split("\n"):
                    if "error" in line.lower() or "fail" in line.lower():
                        QTimer.singleShot(0, lambda line=line: self.log_event("warn", f"Dispatcher alert: {line}"))

        threading.Thread(target=run_cli, daemon=True).start()

    # ─── APP SHUTDOWN OVERRIDES ──────────────────────────────────────────────
    def closeEvent(self, event):
        self.log_event("warn", "Closing Control Dashboard. Sweeping active subprocesses...")
        
        # Stop background ROS worker
        if self.ros_worker:
            self.ros_worker.stop()

        # Stop launches
        self._stop_sim()
        self._stop_rmf()

        # Brief pause to verify clean shutdowns
        time.sleep(1.0)
        event.accept()

    # ─── SLEEK CSS STYLESHEET ────────────────────────────────────────────────
    def _get_stylesheet(self):
        return f"""
        QMainWindow {{
            background-color: {BG_COLOR};
        }}

        QFrame#HeaderFrame {{
            background-color: {PANEL_COLOR};
            border: 1px solid {BORDER_COLOR};
            border-radius: 8px;
        }}

        QGroupBox {{
            background-color: {PANEL_COLOR};
            border: 1px solid {BORDER_COLOR};
            border-radius: 8px;
            margin-top: 15px;
            font-family: 'Inter';
            font-size: 10pt;
            font-weight: bold;
            color: {ACCENT_BLUE};
        }}

        QGroupBox::title {{
            subcontrol-origin: margin;
            subcontrol-position: top left;
            padding: 2px 10px;
            left: 10px;
        }}

        QFrame#RobotCard {{
            background-color: {CARD_COLOR};
            border: 1px solid {BORDER_COLOR};
            border-radius: 6px;
            min-width: 140px;
            max-width: 200px;
        }}

        QFrame#RobotCard:hover {{
            border: 1px solid {ACCENT_LAVENDER};
        }}

        QLabel#RobotCardTitle {{
            color: {TEXT_MAIN};
            border-bottom: 1px solid {BORDER_COLOR};
            padding-bottom: 4px;
        }}

        QLabel#RobotStatusText {{
            padding: 2px;
            border-radius: 4px;
            background-color: rgba(255, 255, 255, 0.03);
        }}

        QProgressBar {{
            border: 1px solid {BORDER_COLOR};
            border-radius: 4px;
            text-align: center;
            background-color: {BG_COLOR};
            color: {TEXT_MAIN};
            font-weight: bold;
            font-size: 8pt;
            max-height: 14px;
        }}

        QProgressBar::chunk {{
            border-radius: 3px;
        }}

        QPushButton {{
            background-color: {CARD_COLOR};
            color: {TEXT_MAIN};
            border: 1px solid {BORDER_COLOR};
            border-radius: 6px;
            padding: 6px 12px;
            font-family: 'Inter';
            font-weight: bold;
            font-size: 9pt;
        }}

        QPushButton:hover {{
            background-color: {BORDER_COLOR};
            border: 1px solid {ACCENT_BLUE};
        }}

        QPushButton:pressed {{
            background-color: {BG_COLOR};
        }}

        QPushButton:disabled {{
            color: {TEXT_MUTED};
            background-color: rgba(255, 255, 255, 0.01);
            border: 1px solid rgba(255, 255, 255, 0.02);
        }}

        QPushButton#PresetButton {{
            background-color: {CARD_COLOR};
            border: 1px solid {BORDER_COLOR};
            color: {ACCENT_CYAN};
        }}

        QPushButton#PresetButton:hover {{
            background-color: {BORDER_COLOR};
            border: 1px solid {ACCENT_CYAN};
        }}

        QPushButton#PresetButtonAccent {{
            background-color: {CARD_COLOR};
            border: 1px solid {BORDER_COLOR};
            color: {ACCENT_LAVENDER};
        }}

        QPushButton#PresetButtonAccent:hover {{
            background-color: {BORDER_COLOR};
            border: 1px solid {ACCENT_LAVENDER};
        }}

        QComboBox, QSpinBox {{
            background-color: {CARD_COLOR};
            color: {TEXT_MAIN};
            border: 1px solid {BORDER_COLOR};
            border-radius: 4px;
            padding: 4px 8px;
            min-height: 25px;
        }}

        QComboBox:hover, QSpinBox:hover {{
            border: 1px solid {ACCENT_BLUE};
        }}

        QTabWidget::pane {{
            border: 1px solid {BORDER_COLOR};
            border-radius: 6px;
            background-color: {PANEL_COLOR};
            top: -1px;
        }}

        QTabBar::tab {{
            background-color: {CARD_COLOR};
            color: {TEXT_MUTED};
            border: 1px solid {BORDER_COLOR};
            border-bottom: none;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
            padding: 6px 12px;
            margin-right: 2px;
            font-family: 'Inter';
            font-size: 8pt;
            font-weight: bold;
        }}

        QTabBar::tab:selected, QTabBar::tab:hover {{
            background-color: {PANEL_COLOR};
            color: {ACCENT_BLUE};
            border-bottom: 1px solid {PANEL_COLOR};
        }}

        QTextEdit {{
            background-color: {BG_COLOR};
            color: {TEXT_MAIN};
            border: 1px solid {BORDER_COLOR};
            border-radius: 6px;
            padding: 8px;
        }}

        QFrame#ConsolePane {{
            background-color: {PANEL_COLOR};
            border: 1px solid {BORDER_COLOR};
            border-radius: 8px;
        }}
        """


# ─── ENTRY POINT ─────────────────────────────────────────────────────────────
def main():
    # Force styling
    QApplication.setStyle("Fusion")
    
    app = QApplication(sys.argv)
    
    # Elegant dark palette override to prevent flashbang windows
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(BG_COLOR))
    palette.setColor(QPalette.WindowText, QColor(TEXT_MAIN))
    palette.setColor(QPalette.Base, QColor(BG_COLOR))
    palette.setColor(QPalette.AlternateBase, QColor(PANEL_COLOR))
    palette.setColor(QPalette.ToolTipBase, QColor(TEXT_MAIN))
    palette.setColor(QPalette.ToolTipText, QColor(TEXT_MAIN))
    palette.setColor(QPalette.Text, QColor(TEXT_MAIN))
    palette.setColor(QPalette.Button, QColor(PANEL_COLOR))
    palette.setColor(QPalette.ButtonText, QColor(TEXT_MAIN))
    palette.setColor(QPalette.Link, QColor(ACCENT_BLUE))
    palette.setColor(QPalette.Highlight, QColor(ACCENT_BLUE))
    palette.setColor(QPalette.HighlightedText, QColor(BG_COLOR))
    app.setPalette(palette)

    window = RMFDashboardWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
