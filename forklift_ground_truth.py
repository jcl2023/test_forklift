#!/usr/bin/env python3

from isaacsim import SimulationApp

# ------------------------------------------------------------
# Start Isaac Sim FIRST
# ------------------------------------------------------------

simulation_app = SimulationApp({
    "headless": True
})

# ------------------------------------------------------------
# Imports after SimulationApp
# ------------------------------------------------------------

import math
import time

import numpy as np

from pxr import Usd, UsdGeom

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# ============================================================
# Configuration
# ============================================================

USD_PATH = (
    "/home/ubuntu/isaac_assets/test_forklift/"
    "forklift_c_camera.usd"
)

ROBOT_PRIM = "/World/forklift_c"

ODOM_TOPIC = "/odom"

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

PUBLISH_HZ = 50.0


# ============================================================
# Quaternion utilities
# ============================================================

def quat_wxyz_to_yaw(q):
    """
    Isaac Sim / USD quaternion convention:

        [w, x, y, z]

    Returns yaw around Z.
    """

    w, x, y, z = q

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


# ============================================================
# Isaac Sim ground-truth ROS 2 publisher
# ============================================================

class ForkliftGroundTruth(Node):

    def __init__(self, stage):

        super().__init__("forklift_ground_truth")

        self.stage = stage

        self.robot_prim = stage.GetPrimAtPath(
            ROBOT_PRIM
        )

        if not self.robot_prim.IsValid():
            raise RuntimeError(
                f"Robot prim not found: {ROBOT_PRIM}"
            )

        self.odom_pub = self.create_publisher(
            Odometry,
            ODOM_TOPIC,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # Previous state for velocity estimation
        self.prev_time = None
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None

        self.timer = self.create_timer(
            1.0 / PUBLISH_HZ,
            self.publish_ground_truth
        )

        self.get_logger().info(
            f"Ground truth publisher started"
        )

        self.get_logger().info(
            f"Robot: {ROBOT_PRIM}"
        )

        self.get_logger().info(
            f"Publishing: {ODOM_TOPIC}"
        )

    # --------------------------------------------------------
    # Get forklift world pose
    # --------------------------------------------------------

    def get_robot_pose(self):

        xform = UsdGeom.Xformable(
            self.robot_prim
        )

        transform = xform.ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

        translation = transform.ExtractTranslation()

        rotation = transform.ExtractRotation()

        quat = rotation.GetQuaternion()

        # Gf.Quatd -> wxyz
        w = quat.GetReal()

        imag = quat.GetImaginary()

        xq = imag[0]
        yq = imag[1]
        zq = imag[2]

        yaw = quat_wxyz_to_yaw(
            [w, xq, yq, zq]
        )

        return (
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
            w,
            xq,
            yq,
            zq,
            yaw
        )

    # --------------------------------------------------------
    # Publish odom + TF
    # --------------------------------------------------------

    def publish_ground_truth(self):

        now = self.get_clock().now()

        (
            x,
            y,
            z,
            qw,
            qx,
            qy,
            qz,
            yaw
        ) = self.get_robot_pose()

        current_time = now.nanoseconds * 1e-9

        # ----------------------------------------------------
        # Estimate velocity from ground truth pose
        # ----------------------------------------------------

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.prev_time is not None:

            dt = current_time - self.prev_time

            if dt > 1e-6:

                vx_world = (
                    x - self.prev_x
                ) / dt

                vy_world = (
                    y - self.prev_y
                ) / dt

                dyaw = yaw - self.prev_yaw

                # Normalize angle
                while dyaw > math.pi:
                    dyaw -= 2.0 * math.pi

                while dyaw < -math.pi:
                    dyaw += 2.0 * math.pi

                wz = dyaw / dt

                # Convert world velocity to base_link
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)

                vx = (
                    cos_yaw * vx_world
                    + sin_yaw * vy_world
                )

                vy = (
                    -sin_yaw * vx_world
                    + cos_yaw * vy_world
                )

        self.prev_time = current_time
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw

        # ----------------------------------------------------
        # Odometry message
        # ----------------------------------------------------

        odom = Odometry()

        odom.header.stamp = now.to_msg()

        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # ----------------------------------------------------
        # TF: odom -> base_link
        # ----------------------------------------------------

        tf = TransformStamped()

        tf.header.stamp = now.to_msg()

        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z

        tf.transform.rotation.w = qw
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz

        self.tf_broadcaster.sendTransform(tf)


# ============================================================
# Main
# ============================================================

def main():

    print("Opening forklift USD...")

    stage = Usd.Stage.Open(USD_PATH)

    if stage is None:
        raise RuntimeError(
            f"Unable to open: {USD_PATH}"
        )

    # Let Isaac Sim initialize the stage
    for _ in range(20):
        simulation_app.update()

    rclpy.init()

    node = None

    try:

        node = ForkliftGroundTruth(stage)

        while simulation_app.is_running():

            # Advance Isaac Sim
            simulation_app.update()

            # Process ROS 2
            rclpy.spin_once(
                node,
                timeout_sec=0.0
            )

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        simulation_app.close()


if __name__ == "__main__":
    main()
