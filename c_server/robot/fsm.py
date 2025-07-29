from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from .arm import arm_pick, arm_place_on_robot
from .nav2_waypoint_class import WaypointNavigator
import config as cfg

from enum import IntEnum, auto

class Step(IntEnum):
    IDLE          = auto()
    GO_TO_ARM     = auto()
    PICK          = auto()
    GO_TO_USER    = auto()
    WAIT_CONFIRM  = auto()
    GO_DOCK       = auto()

# ───────── ROS FSM 정의 ─────────
class DeliveryFSM(Node):
    def __init__(self, robot_name: str, wp_nav: WaypointNavigator):
        super().__init__(f"{robot_name}_delivery_fsm", namespace=robot_name)
        self.step = Step.IDLE
        self.waypoint_nav = wp_nav
        self.create_subscription(String, cfg.ROS_CMD_TOPIC, self.on_cmd, 10)
        self.cmd_pub   = self.create_publisher(String, cfg.ROS_CMD_TOPIC, 10)
        self.stat_pub  = self.create_publisher(String, cfg.ROS_STAT_TOPIC, 10)
        self.create_timer(0.5, self.loop)

    # ---------- 토픽 콜백 / 퍼블리시 ----------
    def on_cmd(self, msg: String):
        if msg.data == "order" and self.step == Step.IDLE:
            self.set_step(Step.GO_TO_ARM)
        elif msg.data == "confirm" and self.step == Step.WAIT_CONFIRM:
            self.set_step(Step.GO_DOCK)

    def pub_status(self, text: str):
        self.stat_pub.publish(String(data=text))
        self.get_logger().info(f"[STATUS] {text}")

    # ---------- 상태 전이 ----------
    def set_step(self, nxt: int):
        self.step = nxt
        self.waypoint_nav.pinky_nav2_state = "None"

        match self.step:
            case Step.IDLE:
                self.pub_status("idle")
            case Step.GO_TO_ARM:
                self.pub_status("moving_to_arm")
                self.waypoint_nav.send_goal(cfg.PICKUP_ST1)
            case Step.PICK:
                self.pub_status("picking")
                arm_pick("vitamin")
                arm_place_on_robot()
                self.set_step(Step.GO_TO_USER)
            case Step.GO_TO_USER:
                self.pub_status("moving_to_user")
                self.waypoint_nav.send_goal(cfg.SERVICE_ST1)
            case Step.WAIT_CONFIRM:
                self.pub_status("waiting_confirm")
            case Step.GO_DOCK:
                self.pub_status("returning_to_dock")
                self.waypoint_nav.send_goal(cfg.CHARGING_ST)

    def loop(self):
        if self.step == Step.GO_TO_ARM  and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.PICK)
        elif self.step == Step.GO_TO_USER and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.WAIT_CONFIRM)
        elif self.step == Step.GO_DOCK   and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.IDLE)
