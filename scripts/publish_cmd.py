from typing import Tuple

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from visualnav_transformer.deployment.src.topic_names import WAYPOINT_TOPIC

# CONSTS
CONFIG_PATH = "config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MIN_V = 0.7 
MAX_ACC_V = 0.5
MAX_ACC_W = np.pi
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"]
DT = 1 / robot_config["frame_rate"]
RATE = 9
EPS = 1e-8
WAYPOINT_TIMEOUT = 1  # seconds # TODO: tune this
FLIP_ANG_VEL = np.pi / 4
LINEAR_VEL = 1e-2

def pd_controller(waypoint: np.ndarray, v_prev: float, w_prev: float) -> Tuple[float, float]:
    """PD controller for the robot with acceleration limiting"""
    assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
    if len(waypoint) == 2:
        dx, dy = waypoint
    else:
        dx, dy, hx, hy = waypoint

    # Compute desired velocities
    if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
        v_des = 0
        w_des = clip_angle(np.arctan2(hy, hx)) / DT
    elif np.abs(dx) < EPS:
        v_des = 0
        w_des = np.sign(dy) * np.pi / (2 * DT)
    else:
        v_des = LINEAR_VEL * dx / np.abs(dy)
        w_des = np.arctan(dy / dx)

    # Acceleration limiting
    max_dv = MAX_ACC_V * DT
    max_dw = MAX_ACC_W * DT
    v = np.clip(v_des, v_prev - max_dv, v_prev + max_dv)
    w = np.clip(w_des, w_prev - max_dw, w_prev + max_dw)

    # Velocity limiting
    v = np.clip(v, -MAX_V, MAX_V)
    w = np.clip(w, -MAX_W, MAX_W)

    # Minimum velocity enforcement
    if np.abs(v) > 0 and np.abs(v) < MIN_V:
        v = np.sign(v) * MIN_V

    return v, w

class VelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher")

        self.waypoint_subscription = self.create_subscription(
            Float32MultiArray, WAYPOINT_TOPIC, self.waypoint_callback, 10
        )

        self.publisher = self.create_publisher(Twist, "/joey1/cmd_vel", 10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_waypoint = None

        # Initialize previous velocities
        self.v_prev = 0.0
        self.w_prev = 0.0

    def waypoint_callback(self, msg):
        v, w = pd_controller(np.array(msg.data), self.v_prev, self.w_prev)

        # Store for next time step
        self.v_prev = v
        self.w_prev = w

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
