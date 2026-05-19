#!/usr/bin/env python3
"""
RMF End-to-End Pipeline Tracer
Monitors the complete communication chain from Gazebo → Nav2 → Fleet Adapter
→ Fleet Manager → RMF Core → Bidding → Task Dispatch → Robot Execution
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# ROS 2 message types
from rcl_interfaces.msg import Log
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rmf_fleet_msgs.msg import FleetState
from rmf_task_msgs.msg import BidNotice, BidResponse, TaskSummary, ApiRequest, ApiResponse, DispatchCommand, DispatchAck, DispatchStates
from std_msgs.msg import Bool

import requests
import threading
import time
import sys
import os
from datetime import datetime
from collections import deque

# ─── ANSI Color Codes ────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
RESET  = "\033[0m"

def ok(msg):    return f"{GREEN}[OK]  {RESET} {msg}"
def fail(msg):  return f"{RED}[FAIL]{RESET} {msg}"
def warn(msg):  return f"{YELLOW}[WARN]{RESET} {msg}"
def info(msg):  return f"{CYAN}[INFO]{RESET} {msg}"
def ts():       return datetime.now().strftime("%H:%M:%S")

PIPELINE_STEPS = [
    ("Request",    "/task_api_requests"),
    ("Bidding",    "/rmf_task/bid_notice"),
    ("Bids",       "/rmf_task/bid_response"),
    ("Award",      "/rmf_task/dispatch_request"),
    ("Accept",     "/rmf_task/dispatch_ack"),
    ("Running",    "/task_summaries (ACTIVE)"),
    ("Complete",   "/task_summaries (COMPLETED)")
]


class PipelineTracer(Node):
    """Live tracer node for the entire RMF pipeline."""

    FLEET_NAME = "dynominion_fleet"
    ROBOTS     = ["dynominion1", "dynominion2", "dynominion3", "dynominion4", "dynominion5"]
    FM_HOST    = "http://localhost:8080"

    def __init__(self):
        super().__init__("rmf_pipeline_tracer")

        # ── Stage status booleans ─────────────────────────────────────────
        self.gazebo_ok           = False
        self.rmf_core_ok         = False
        self.fleet_manager_ok    = False
        self.task_dispatcher_ok  = False

        # Flips to True once infra + all robots are confirmed — after that
        # only RMF task-flow events are written to the log.
        self._system_healthy     = False

        self.nav2_ready    = {r: False for r in self.ROBOTS}
        self.rmf_reg       = {r: False for r in self.ROBOTS}
        self.fm_api_state  = {}   # robot_name -> latest state dict from HTTP

        # ── Bidding state ─────────────────────────────────────────────────
        self.bid_notices   = {}   # task_id -> {time, request_type}
        self.bid_responses = {}   # task_id -> [list of {fleet, robot, time}]

        # ── Task lifecycle ─────────────────────────────────────────────────
        self.tasks         = {}   # task_id -> {state, assigned_to, phase, time}

        # ── Navigation feedback ───────────────────────────────────────────
        self.nav_goals     = {r: None for r in self.ROBOTS}   # robot -> last goal time

        # ── Dispatch ID mapping ───────────────────────────────────────────
        self.dispatch_to_task = {}  # dispatch_id -> task_id

        # ── Pipeline Progress ─────────────────────────────────────────────
        self.task_pipelines = {}    # task_id -> last_step_index
        self.latest_tid     = None

        # ── Event log (rolling, most-recent first) ────────────────────────
        self.log           = deque(maxlen=40)

        # ── Lock for thread-safe log writes ──────────────────────────────
        self._lock = threading.Lock()

        self._setup_subscriptions()
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)  # suppress rclpy noise

    # ─────────────────────────────────────────────────────────────────────
    # Subscription setup
    # ─────────────────────────────────────────────────────────────────────

    def _setup_subscriptions(self):
        # transient_qos = QoSProfile(
        #     depth=10,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     reliability=ReliabilityPolicy.RELIABLE
        # )

        # Gazebo clock
        self.create_subscription(Bool, "/world/new_env/clock",
            lambda _: None, 10)   # existence check done via topic list

        # RMF Traffic heartbeat → RMF core health
        self.create_subscription(Bool, "/rmf_traffic/heartbeat",
            self._rmf_heartbeat_cb, 10)

        # Fleet states → robot registration
        self.create_subscription(FleetState, "/fleet_states",
            self._fleet_state_cb, 10)

        # Bidding
        self.create_subscription(BidNotice, "/rmf_task/bid_notice",
            self._bid_notice_cb, 10)
        self.create_subscription(BidResponse, "/rmf_task/bid_response",
            self._bid_response_cb, 10)

        # Task dispatch
        self.create_subscription(DispatchCommand, "/rmf_task/dispatch_request",
            self._dispatch_request_cb, 10)
        self.create_subscription(DispatchAck, "/rmf_task/dispatch_ack",
            self._dispatch_ack_cb, 10)
        self.create_subscription(DispatchStates, "/rmf_task/dispatch_states",
            self._dispatch_states_cb, 10)

        # Task lifecycle
        self.create_subscription(TaskSummary, "/task_summaries",
            self._task_summary_cb, 10)

        # Task API
        self.create_subscription(ApiRequest, "/task_api_requests",
            self._api_request_cb, 10)
        self.create_subscription(ApiResponse, "/task_api_responses",
            self._api_response_cb, 10)



    # ─────────────────────────────────────────────────────────────────────
    # ROS 2 Callbacks
    # ─────────────────────────────────────────────────────────────────────

    def _rmf_heartbeat_cb(self, _msg):
        self.rmf_core_ok = True

    def _fleet_state_cb(self, msg):
        if msg.name != self.FLEET_NAME:
            return
        self.task_dispatcher_ok = True  # fleet states mean adapter is live
        for robot in msg.robots:
            if robot.name in self.rmf_reg:
                was_reg = self.rmf_reg[robot.name]
                self.rmf_reg[robot.name] = True
                if not was_reg and not self._system_healthy:
                    self._log(ok(f"Robot [{robot.name}] → registered to RMF fleet  "
                                 f"(Publisher: FleetAdapter → Topic: /fleet_states)"))



    def _bid_notice_cb(self, msg):
        tid = msg.task_id if msg.task_id else "unknown"
        self.bid_notices[tid] = {"time": ts()}
        self.latest_tid = tid
        self.task_pipelines[tid] = 1 # Bidding
        self._log(ok(f"Bid Notice  Task:[{tid}]  "
                     f"(Publisher: rmf_task_dispatcher → /rmf_task/bid_notice)"))

    def _bid_response_cb(self, msg):
        tid = msg.task_id
        if tid not in self.bid_responses:
            self.bid_responses[tid] = []
        fleet = msg.proposal.fleet_name        if msg.has_proposal else "(no proposal)"
        robot = msg.proposal.expected_robot_name if msg.has_proposal else "(no proposal)"
        entry = {"fleet": fleet, "robot": robot, "time": ts()}
        self.bid_responses[tid].append(entry)
        if tid in self.task_pipelines:
            self.task_pipelines[tid] = max(self.task_pipelines[tid], 2) # Bids
        self._log(ok(f"Bid Response  Task:[{tid}]  "
                     f"Fleet:[{fleet}]  Robot:[{robot}]  "
                     f"(Publisher: FleetAdapter → /rmf_task/bid_response)"))

    def _dispatch_request_cb(self, msg):
        tid   = msg.task_id if hasattr(msg, 'task_id') else "unknown"
        did   = msg.dispatch_id if hasattr(msg, 'dispatch_id') else 0
        fleet = msg.fleet_name if hasattr(msg, 'fleet_name') else "?"
        if did:
            self.dispatch_to_task[did] = tid
        if tid != "unknown":
            self.latest_tid = tid
            self.task_pipelines[tid] = max(self.task_pipelines.get(tid, 0), 3) # Award
        self._log(ok(f"Dispatch Command  Task:[{tid}]  Fleet:[{fleet}]  "
                     f"(Publisher: rmf_task_dispatcher → /rmf_task/dispatch_command)"))

    def _dispatch_states_cb(self, msg):
        for state in msg.states:
            tid   = state.task_profile.task_id if hasattr(state, 'task_profile') else "?"
            fleet = state.fleet_name if hasattr(state, 'fleet_name') else "?"
            self._log(info(f"Dispatch State update → Fleet:[{fleet}] Task:[{tid}]  "
                           f"(Topic: /rmf_task/dispatch_states)"))

    def _dispatch_ack_cb(self, msg):
        success = msg.errors == [] if hasattr(msg, 'errors') else True
        did     = msg.dispatch_id if hasattr(msg, 'dispatch_id') else 0
        tid     = self.dispatch_to_task.get(did, "unknown")
        if success and tid != "unknown":
            self.task_pipelines[tid] = max(self.task_pipelines.get(tid, 0), 4) # Accept
        tag = ok if success else fail
        self._log(tag(f"Dispatch Ack  Task:[{tid}]  success={success}  "
                      f"(Publisher: FleetAdapter → /rmf_task/dispatch_ack)"))

    def _task_summary_cb(self, msg):
        STATE_NAMES = {
            0: "QUEUED", 1: "SELECTED", 2: "DISPATCHED",
            3: "ACTIVE", 4: "FAILED", 5: "COMPLETED",
            6: "PENDING", 7: "CANCELLED"
        }
        state_name = STATE_NAMES.get(msg.state, f"STATE_{msg.state}")
        tid = msg.task_id
        robot = msg.robot_name if hasattr(msg, 'robot_name') else "?"
        prev = self.tasks.get(tid, {}).get("state")
        self.tasks[tid] = {
            "state": msg.state, "state_name": state_name,
            "robot": robot, "time": ts()
        }
        if prev != msg.state:
            if msg.state == 3:  # ACTIVE
                if tid in self.task_pipelines:
                    self.task_pipelines[tid] = max(self.task_pipelines[tid], 5) # Running
                self._log(ok(f"Task [{tid}] → ACTIVE on robot [{robot}]  "
                             f"(Publisher: rmf_task_dispatcher → /task_summaries)"))
            elif msg.state == 5:  # COMPLETED
                if tid in self.task_pipelines:
                    self.task_pipelines[tid] = 6 # Complete
                self._log(ok(f"Task [{tid}] → COMPLETED by robot [{robot}]  ✓"))
            elif msg.state == 4:  # FAILED
                self._log(fail(f"Task [{tid}] → FAILED on robot [{robot}]"))
            else:
                self._log(info(f"Task [{tid}] → {state_name}  robot=[{robot}]"))

    def _api_request_cb(self, msg):
        self._log(info(f"Task API Request received  "
                       f"(Publisher: external client → /task_api_requests)"))

    def _api_response_cb(self, msg):
        success = "success" in (msg.json if hasattr(msg, 'json') else "").lower()
        tag = ok if success else warn
        self._log(tag(f"Task API Response sent  "
                      f"(Publisher: rmf_task_dispatcher → /task_api_responses)"))

    def check_gazebo(self):
        topics = [t for t, _ in self.get_topic_names_and_types()]
        was = self.gazebo_ok
        self.gazebo_ok = "/world/new_env/clock" in topics or "/clock" in topics
        if self.gazebo_ok and not was and not self._system_healthy:
            self._log(ok("Gazebo simulation started  "
                         "(Publisher: gzserver → /world/new_env/clock)"))

    def check_rmf_core(self):
        topics = [t for t, _ in self.get_topic_names_and_types()]
        was = self.rmf_core_ok
        self.rmf_core_ok = "/rmf_traffic/participants" in topics
        if self.rmf_core_ok and not was and not self._system_healthy:
            self._log(ok("RMF Core running  "
                         "(Node: rmf_traffic_schedule → /rmf_traffic/participants)"))

    def check_fleet_manager(self):
        was = self.fleet_manager_ok
        try:
            r = requests.get(f"{self.FM_HOST}/v1/robots/{self.ROBOTS[0]}/state", timeout=0.3)
            self.fleet_manager_ok = r.status_code == 200
            if self.fleet_manager_ok:
                for robot in self.ROBOTS:
                    try:
                        res = requests.get(f"{self.FM_HOST}/v1/robots/{robot}/state", timeout=0.3)
                        if res.status_code == 200:
                            self.fm_api_state[robot] = res.json()
                    except Exception: pass
        except Exception:
            self.fleet_manager_ok = False

        if self.fleet_manager_ok and not was and not self._system_healthy:
            self._log(ok(f"Fleet Manager HTTP API online  (Server: fleet_manager_node → {self.FM_HOST})"))
        elif not self.fleet_manager_ok and was:
            self._log(fail(f"Fleet Manager HTTP API went offline!"))

    def check_nav2(self):
        topics = [t for t, _ in self.get_topic_names_and_types()]
        for robot in self.ROBOTS:
            was = self.nav2_ready[robot]
            self.nav2_ready[robot] = f"/{robot}/amcl_pose" in topics
            if self.nav2_ready[robot] and not was and not self._system_healthy:
                self._log(ok(f"[{robot}] Nav2 AMCL topic active  "
                             f"(Publisher: amcl → Topic: /{robot}/amcl_pose)"))

    def _log(self, message):
        with self._lock:
            self.log.appendleft(f"  {ts()}  {message}")

    def _refresh_health(self):
        if not self._system_healthy:
            if (self.gazebo_ok and self.rmf_core_ok and self.fleet_manager_ok and all(self.rmf_reg.values())):
                self._system_healthy = True
                self._log(ok(f"{BOLD}System verified HEALTHY — log now focused on RMF task events only{RESET}"))

    def render(self):
        self._refresh_health()
        lines = []
        W = 92
        def sep(char="─"): return char * W
        def header(t): return f"{BOLD}{t:^{W}}{RESET}"

        lines.append(sep("═"))
        lines.append(header("  RMF END-TO-END PIPELINE TRACER  (live)  "))
        lines.append(sep("═"))

        def dot(v): return f"{GREEN}✅{RESET}" if v else f"{RED}❌{RESET}"
        lines.append(f"  {BOLD}INFRASTRUCTURE{RESET}")
        lines.append(f"  Gazebo:        {dot(self.gazebo_ok)}")
        lines.append(f"  RMF Core:      {dot(self.rmf_core_ok)}")
        lines.append(f"  Fleet Manager: {dot(self.fleet_manager_ok)}")
        lines.append(sep())

        lines.append(f"  {BOLD}ROBOT PIPELINE STATUS{RESET}  {DIM}(Nav2 AMCL | RMF Reg | FM State | FM Task){RESET}")
        for robot in self.ROBOTS:
            n2  = dot(self.nav2_ready[robot])
            reg = dot(self.rmf_reg[robot])
            api = self.fm_api_state.get(robot, {})
            fm_state = api.get("state", "—")
            fm_task  = api.get("task_id") or "—"
            fm_color = GREEN if fm_state == "IDLE" else (YELLOW if fm_state == "NAVIGATING" else RED)
            lines.append(f"  {robot:<14} Nav2:{n2}  RMF:{reg}  FleetMgr:[{fm_color}{fm_state:<11}{RESET}]  Task:{DIM}{fm_task}{RESET}")
        lines.append(sep())

        lines.append(f"  {BOLD}BIDDING & DISPATCH{RESET}")
        valid_notices = {k: v for k, v in self.bid_notices.items() if k != "unknown"}
        if valid_notices:
            sorted_tids = sorted(valid_notices.keys(), key=lambda k: valid_notices[k]['time'], reverse=True)[:3]
            for tid in sorted_tids:
                data = valid_notices[tid]
                responses = self.bid_responses.get(tid, [])
                lines.append(f"  {CYAN}BidNotice{RESET}  Task:[{tid}]  @{data['time']}")
                for r in responses:
                    lines.append(f"    {GREEN}┕ BidResponse{RESET}  Robot:[{r['robot']}]  Fleet:[{r['fleet']}]")
        else: lines.append(f"  {DIM}No bidding activity observed yet.{RESET}")
        lines.append(sep())

        lines.append(f"  {BOLD}TASK LIFECYCLE{RESET}")
        STATE_COLOR = {"QUEUED": CYAN, "SELECTED": CYAN, "DISPATCHED": YELLOW, "ACTIVE": GREEN, "COMPLETED": GREEN, "FAILED": RED, "CANCELLED": RED}
        if self.tasks:
            for tid, data in list(self.tasks.items())[-5:]:
                sc = STATE_COLOR.get(data["state_name"], RESET)
                lines.append(f"  {DIM}{tid:<35}{RESET}  [{sc}{data['state_name']:<11}{RESET}]  Robot:{data['robot']:<14}  @{data['time']}")
        else: lines.append(f"  {DIM}No tasks observed yet.{RESET}")
        lines.append(sep())

        lines.append(f"  {BOLD}TASK PIPELINE PROGRESS{RESET}")
        if self.latest_tid and self.latest_tid in self.task_pipelines:
            tid = self.latest_tid
            current_step = self.task_pipelines[tid]
            pipeline_viz = []
            for i, (name, topic) in enumerate(PIPELINE_STEPS):
                if i < current_step: pipeline_viz.append(f"{GREEN}{name}{RESET}")
                elif i == current_step: pipeline_viz.append(f"{YELLOW}{BOLD}{name}{RESET}")
                else: pipeline_viz.append(f"{DIM}{name}{RESET}")
            lines.append(f"  Task:[{CYAN}{tid}{RESET}]")
            lines.append("  " + " → ".join(pipeline_viz))
            if current_step < len(PIPELINE_STEPS) - 1:
                next_step = PIPELINE_STEPS[current_step + 1]
                lines.append(f"  {YELLOW}NEXT EXPECTED:{RESET} {BOLD}{next_step[0]}{RESET} on {DIM}{next_step[1]}{RESET}")
            else: lines.append(f"  {GREEN}Pipeline Complete ✓{RESET}")
        else: lines.append(f"  {DIM}No active pipeline to track.{RESET}")
        lines.append(sep())

        if self._system_healthy: log_title = f"{BOLD}RMF TASK EVENT LOG{RESET}  {DIM}(bidding / dispatch / lifecycle — newest first){RESET}"
        else: log_title = f"{BOLD}STARTUP EVENT LOG{RESET}  {DIM}(newest first){RESET}"
        lines.append(f"  {log_title}")
        for entry in list(self.log)[:20]: lines.append(entry)
        lines.append(sep("═"))
        
        all_nav, all_reg = all(self.nav2_ready.values()), all(self.rmf_reg.values())
        infra_ok = self.gazebo_ok and self.rmf_core_ok and self.fleet_manager_ok
        if infra_ok and all_nav and all_reg: lines.append(f"  {GREEN}{BOLD}SYSTEM STATUS: HEALTHY — Ready to receive tasks{RESET}")
        else:
            lines.append(f"  {RED}{BOLD}SYSTEM STATUS: INCOMPLETE{RESET}")
            if not self.gazebo_ok: lines.append(f"    {RED}▶ Gazebo not detected{RESET}")
            if not self.rmf_core_ok: lines.append(f"    {RED}▶ RMF Core not running{RESET}")
            if not self.fleet_manager_ok: lines.append(f"    {RED}▶ Fleet Manager offline{RESET}")
        lines.append(sep("═"))
        lines.append(f"  {DIM}Press Ctrl+C to exit  |  Last refresh: {ts()}{RESET}\n")

        print("\033[H\033[J", end="")
        print("\n".join(lines))

def main():
    rclpy.init()
    tracer = PipelineTracer()
    try:
        while rclpy.ok():
            tracer.check_gazebo(); tracer.check_rmf_core(); tracer.check_fleet_manager(); tracer.check_nav2()
            for _ in range(5): rclpy.spin_once(tracer, timeout_sec=0.1)
            tracer.render()
            time.sleep(0.5)
    except KeyboardInterrupt: pass
    tracer.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
