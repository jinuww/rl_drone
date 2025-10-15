#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with ArUco detection."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # OpenCV window for ArUco
        cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        self.latest_depth = None

        # Subscribe to color image topic for ArUco detection
        self.color_subscriber = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',  # Replace with actual topic
            self.color_image_callback,
            qos_profile_sensor_data
        )

        # QoS profile for control topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers for vehicle state
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)

        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -8.0
        # 0: pre-offboard, 1: takeoff, 2: go to (0.0, 3.0, -1.0), 3: (0.65, 3.0, 0.0), 4:land
        self.phase = 0

        # Timer for periodic control
        self.timer = self.create_timer(0.1, self.timer_callback)

    def color_image_callback(self, msg: Image):
        """Callback for color images: detect ArUco tags and display."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Color CvBridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco detection
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners[0]
                cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].mean())
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # If depth available, measure
                if self.latest_depth is not None:
                    d = float(self.latest_depth[cy, cx])
                    self.get_logger().info(f"ArUco ID={int(marker_id)} at ({cx},{cy}), Distance={d:.2f} m")
                else:
                    self.get_logger().info(f"ArUco ID={int(marker_id)} at ({cx},{cy}), no depth info")

        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True; msg.velocity = False
        msg.acceleration = False; msg.attitude = False; msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing setpoint: {[x,y,z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1; msg.target_component = 1
        msg.source_system = 1; msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        # Phase logic...
        if self.phase == 0:
            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode(); self.arm()
                self.phase = 1
            else:
                self.offboard_setpoint_counter += 1
            return

        if self.phase == 1:
            dx1 = abs(self.vehicle_local_position.x - 0.0)
            dy1 = abs(self.vehicle_local_position.y - 0.0)
            dz1 = abs(self.vehicle_local_position.z - self.takeoff_height)
            if dx1 < 0.1 and dy1 < 0.1 and dz1 < 0.1:
                self.phase = 2
            else:
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            return

        if self.phase == 2:
            dx2 = abs(self.vehicle_local_position.x - 0.0)
            dy2 = abs(self.vehicle_local_position.y - 3.0)
            dz2 = abs(self.vehicle_local_position.z + 0.0)
            if dx2 < 0.1 and dy2 < 0.1 and dz2 < 0.1:
                self.phase = 3
            else:
                self.publish_position_setpoint(0.0, 3.0, 0.0)
            return

        if self.phase == 3:
            self.land()
            self.get_logger().info("Landing initiated, exiting.")
            sys.exit(0)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
