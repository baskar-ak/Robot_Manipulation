#!/usr/bin/env python3
"""
BT Sequence:
  1. OpenGripper
  2. Grabbing    - detect cup, adjust pan, pregrasp, grasp, close
  3. AttachCube  (PROVIDED)
  4. MoveToBox   - lift, move over bin
  5. DetachCube  (PROVIDED)
  6. OpenGripper
"""

import time
import math
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import py_trees
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration as DurationMsg

import numpy as np
import cv2


# Camera calibration (verified zero error)
CAM_WORLD_X =  0.05
CAM_WORLD_Y =  0.03
IMG_CX = 640.0
IMG_CY = 360.0
SCALE_X = (0.2364 - CAM_WORLD_X) / (773.0 - IMG_CX)
SCALE_Y = (-0.3100 - CAM_WORLD_Y) / (239.0 - IMG_CY)

BASE_WORLD_X = -0.1347
BASE_WORLD_Y =  0.0


# Joint poses tuned from RViz
# pan overridden at runtime from camera detection
JOINTS_PREGRASP = [-0.3, 0.3,  0.544,  0.024, 1.555]
# At cup grip height — lower shoulder_lift to descend onto cup
JOINTS_GRASP    = [-0.3,  0.3,   0.544,  0.024, 1.555]
# Safe lift after grasping — arm raised before swinging to bin side
JOINTS_LIFT     = [-0.3,   0.3,  0.544,  0.024, 1.555]
# Over the bin opening — tuned from RViz
JOINTS_BOX      = [ 0.658, -1.0,  0.667,  0.058, 1.555]
# Inside the bin — lower arm slightly to deposit cup gently
JOINTS_BOX_LOWER = [ 0.658,  -0.7,  0.667,  0.058, 1.555]

# Gripper
GRIPPER_OPEN  = 1.7453
GRIPPER_CLOSE = 0.35

# ROS interfaces
ARM_ACTION     = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
ATTACH_TOPIC   = "/isaac_attach_cube"
ARM_DURATION   = 3.0
GRIP_DURATION  = 1.5

# Red HSV detection
RED_RANGES = [
    ((0,   120, 70), (10,  255, 255)),
    ((170, 120, 70), (180, 255, 255)),
]
MIN_BLOB_AREA = 500

# Robot interface — non-blocking action client
class RobotInterface:
    def __init__(self, node: Node):
        self.node = node
        self._arm     = ActionClient(node, FollowJointTrajectory, ARM_ACTION)
        self._gripper = ActionClient(node, FollowJointTrajectory, GRIPPER_ACTION)
        self._done = False
        self._ok   = False
        self._lock = threading.Lock()

    def wait_for_servers(self, timeout=5.0) -> bool:
        return (self._arm.wait_for_server(timeout_sec=timeout) and
                self._gripper.wait_for_server(timeout_sec=timeout))

    def _traj(self, joint_names, positions, duration):
        traj = JointTrajectory()
        traj.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions  = list(positions)
        pt.velocities = [0.0] * len(positions)
        s = int(duration)
        pt.time_from_start = DurationMsg(sec=s,
            nanosec=int((duration - s) * 1e9))
        traj.points = [pt]
        g = FollowJointTrajectory.Goal()
        g.trajectory = traj
        return g

    def _reset(self):
        with self._lock:
            self._done = False
            self._ok   = False

    def _on_result(self, f):
        with self._lock:
            try:
                self._ok = (f.result().status == 4)
            except Exception:
                self._ok = False
            self._done = True

    def _on_accept(self, f):
        with self._lock:
            try:
                gh = f.result()
                if not gh.accepted:
                    self._done = True
                    self._ok   = False
                    return
                gh.get_result_async().add_done_callback(self._on_result)
            except Exception:
                self._done = True
                self._ok   = False

    def _send(self, client, goal):
        self._reset()
        client.send_goal_async(goal).add_done_callback(self._on_accept)

    def arm(self, positions, duration=ARM_DURATION):
        names = ["shoulder_pan","shoulder_lift","elbow_flex",
                 "wrist_flex","wrist_roll"]
        self.node.get_logger().info(
            f"Arm → {[round(v,3) for v in positions]}")
        self._send(self._arm, self._traj(names, positions, duration))

    def grip(self, position, duration=GRIP_DURATION):
        self.node.get_logger().info(f"Gripper → {position:.3f}")
        self._send(self._gripper, self._traj(["gripper"], [position], duration))

    def done(self) -> Optional[bool]:
        with self._lock:
            if not self._done:
                return None
            return self._ok

# Red cup detector — returns shoulder_pan angle to face cup
class RedCupDetector:
    def __init__(self, node: Node):
        self._img  = None
        self._lock = threading.Lock()
        node.create_subscription(Image, "/rgb", self._cb, 1)

    def _cb(self, msg):
        with self._lock:
            self._img = msg

    def detect_pan(self) -> Optional[float]:
        """
        Returns shoulder_pan angle (rad) to face the red cup, or None.
        Uses live camera feed — recomputed every call.
        """
        with self._lock:
            msg = self._img
        if msg is None:
            return None

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        bgr = arr[:,:,::-1].copy() if msg.encoding == "rgb8" else arr.copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in RED_RANGES:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < MIN_BLOB_AREA:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        # Pixel → world XY → base_link XY
        wx = CAM_WORLD_X + (u - IMG_CX) * SCALE_X
        wy = CAM_WORLD_Y + (v - IMG_CY) * SCALE_Y
        bx = wx - BASE_WORLD_X
        by = wy - BASE_WORLD_Y

        pan = math.atan2(by, bx)
        pan = max(-1.91, min(1.91, pan))
        return pan

# BT Leaves
class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node  = node
        self.robot = robot
        self._sent = False

    def initialise(self):
        self._sent = False

    def update(self):
        if not self._sent:
            self.node.get_logger().info(f"[{self.name}] open")
            self.robot.grip(GRIPPER_OPEN)
            self._sent = True
            return py_trees.common.Status.RUNNING
        r = self.robot.done()
        if r is None: return py_trees.common.Status.RUNNING
        return (py_trees.common.Status.SUCCESS if r
                else py_trees.common.Status.FAILURE)

class Grabbing(py_trees.behaviour.Behaviour):
    """
    1. Detect red cup -> get shoulder_pan angle from camera
    2. Move to pregrasp
    3. Move to grasp
    4. Close gripper
    """
    def __init__(self, name, node, robot, detector):
        super().__init__(name)
        self.node     = node
        self.robot    = robot
        self.detector = detector
        self._state   = "DETECT"
        self._tries   = 0
        self._pan     = None

    def initialise(self):
        self._state = "DETECT"
        self._tries = 0
        self._pan   = None

    def update(self):
        if self._state == "DETECT":
            self._tries += 1
            pan = self.detector.detect_pan()
            if pan is None:
                if self._tries < 20:
                    return py_trees.common.Status.RUNNING
                self.node.get_logger().error(f"[{self.name}] cup not found")
                return py_trees.common.Status.FAILURE
            self._pan = pan
            self.node.get_logger().info(
                f"[{self.name}] cup detected, pan={pan:.3f} rad")
            self._state = "PREGRASP"
            return py_trees.common.Status.RUNNING

        if self._state == "PREGRASP":
            pose = list(JOINTS_PREGRASP)
            pose[0] = self._pan   # override pan with camera-derived value
            self.robot.arm(pose)
            self._state = "PREGRASP_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "PREGRASP_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            if not r:
                self.node.get_logger().error(f"[{self.name}] pregrasp failed")
                return py_trees.common.Status.FAILURE
            self._state = "GRASP"
            return py_trees.common.Status.RUNNING

        if self._state == "GRASP":
            pose = list(JOINTS_GRASP)
            pose[0] = self._pan
            self.robot.arm(pose)
            self._state = "GRASP_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "GRASP_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            if not r:
                self.node.get_logger().error(f"[{self.name}] grasp failed")
                return py_trees.common.Status.FAILURE
            self._state = "CLOSE"
            return py_trees.common.Status.RUNNING

        if self._state == "CLOSE":
            self.robot.grip(GRIPPER_CLOSE)
            self._state = "CLOSE_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "CLOSE_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            if r:
                self.node.get_logger().info(f"[{self.name}] SUCCESS")
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

class MoveToBox(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node  = node
        self.robot = robot
        self._state = "LIFT"

    def initialise(self):
        self._state = "LIFT"

    def update(self):
        if self._state == "LIFT":
            self.robot.arm(JOINTS_LIFT)
            self._state = "LIFT_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "LIFT_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            self._state = "BOX"
            return py_trees.common.Status.RUNNING

        if self._state == "BOX":
            self.robot.arm(JOINTS_BOX)
            self._state = "BOX_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "BOX_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            if not r:
                self.node.get_logger().error(f"[{self.name}] BOX FAILED")
                return py_trees.common.Status.FAILURE
            self._state = "BOX_LOWER"
            return py_trees.common.Status.RUNNING

        if self._state == "BOX_LOWER":
            self.node.get_logger().info(f"[{self.name}] Lowering into bin")
            self.robot.arm(JOINTS_BOX_LOWER)
            self._state = "BOX_LOWER_WAIT"
            return py_trees.common.Status.RUNNING

        if self._state == "BOX_LOWER_WAIT":
            r = self.robot.done()
            if r is None: return py_trees.common.Status.RUNNING
            if r:
                self.node.get_logger().info(f"[{self.name}] SUCCESS")
                return py_trees.common.Status.SUCCESS
            self.node.get_logger().error(f"[{self.name}] Lower FAILED")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

class AttachDetach(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, attach):
        super().__init__(name)
        self.pub   = node.create_publisher(Bool, ATTACH_TOPIC, 10)
        self.attach = attach
        self._start = None

    def initialise(self):
        self._start = time.monotonic()

    def update(self):
        if time.monotonic() - self._start >= 0.5:
            msg = Bool()
            msg.data = self.attach
            self.pub.publish(msg)
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

# Tree
def create_tree(node, robot, detector):
    seq = py_trees.composites.Sequence(name="PickAndPlace", memory=True)
    seq.add_children([
        py_trees.decorators.Retry("R1",
            OpenGripper("OpenGripper1", node, robot), 2),
        py_trees.decorators.Retry("R2",
            Grabbing("Grabbing", node, robot, detector), 2),
        AttachDetach("Attach", node, attach=True),
        py_trees.decorators.Retry("R3",
            MoveToBox("MoveToBox", node, robot), 2),
        AttachDetach("Detach", node, attach=False),
        py_trees.decorators.Retry("R4",
            OpenGripper("OpenGripper2", node, robot), 2),
    ])
    return py_trees.decorators.OneShot("RunOnce", seq,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)

# Node
class BTNode(Node):
    def __init__(self):
        super().__init__("bt_node")
        self._robot    = RobotInterface(self)
        self._detector = RedCupDetector(self)
        self._tree     = None
        self._start_t  = self.create_timer(1.0, self._check)
        self.get_logger().info("BTNode ready — waiting for controllers")

    def _check(self):
        if self._robot.wait_for_servers(timeout=0.1):
            self._start_t.cancel()
            self.get_logger().info("Controllers ready — starting BT")
            self._tree = py_trees.trees.BehaviourTree(
                create_tree(self, self._robot, self._detector))
            self.create_timer(0.1, self._tick)
        else:
            self.get_logger().info("Waiting for controllers…")

    def _tick(self):
        if self._tree:
            self._tree.tick()

def main():
    rclpy.init()
    node = BTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
