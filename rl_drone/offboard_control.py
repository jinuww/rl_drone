import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)
import subprocess
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import random

class OffboardControlForRL(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_rl_reset')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp   = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd  = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_image_overlay = self.create_publisher(Image, '/depth/image_with_overlay', qos_profile_sensor_data)

        self.sub_pos  = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.cb_pos, qos)
        self.sub_stat = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.cb_stat, qos)

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

        self.pos    = VehicleLocalPosition()
        self.stat   = VehicleStatus()
        self.counter = -1
        self.armed = False
        self.offboard_enabled = False

        self.episode_start_time = None
        self.cooldown_start_time = None
        self.takeoff_z = -5.0
        self.hover_yaw = 0.0
        self.episode_duration = 10.0
        self.cooldown_duration = 10.0

        self.local_position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.tracking = [320.0, 240.0, 1.0]

        self.timer = self.create_timer(0.05, self.timer_callback)

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
                # if self.latest_depth is not None:
                #     d = float(self.latest_depth[cy, cx])
                #     self.get_logger().info(f"ArUco ID={int(marker_id)} at ({cx},{cy}), Distance={d:.2f} m")
                # else:
                #     self.get_logger().info(f"ArUco ID={int(marker_id)} at ({cx},{cy}), no depth info")

        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)

    def get_state_vector(self):
        """
        드론의 현재 상태를 벡터로 반환
        - 위치 (x, y, z)
        - 속도 (vx, vy, vz)
        - 이미지 트래킹 (cx, cy, distance)
        총 9차원 벡터 반환 (상황에 따라 차원 조정 가능)
        """
        pos = self.local_position if self.local_position is not None else [0.0, 0.0, 0.0]
        vel = self.velocity if self.velocity is not None else [0.0, 0.0, 0.0]
        tracking = self.tracking if self.tracking is not None else [320.0, 240.0, 1.0]

        return pos + vel + tracking

    def cb_pos(self, msg):
        self.pos = msg

    def cb_stat(self, msg):
        self.stat = msg

    def arm(self):
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('🟢 ARMED')
        self.armed = True

    def disarm(self):
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('🔴 DISARMED')
        self.armed = False

    def engage_offboard(self):
        for _ in range(10):  # 약 0.5초
            self._publish_position_setpoint(self.takeoff_z)
            time.sleep(0.05)

        self._publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('🔁 OFFBOARD MODE ENABLED')

        self._publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('🟢 ARMED')

        self.offboard_enabled = True
        self.armed = True

    def _publish_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_mode.publish(msg)

    def _publish_position_setpoint(self, z):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -5.0]  # float()으로 강제 변환
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = float(self.hover_yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_sp.publish(msg)

    def _publish_cmd(self, cmd, **p):
        msg = VehicleCommand()
        msg.command = cmd
        for i, param in enumerate(['param1','param2','param3','param4','param5','param6','param7']):
            setattr(msg, param, p.get(param, 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(msg)

    def reset_drone_position(self):
        # PX4 Reboot
        self._publish_cmd(VehicleCommand.VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1.0)
        self.get_logger().info("🔄 PX4 Reboot 명령 전송됨. 센서 안정화 대기 중 (10초)...")
        time.sleep(10)

        # Gazebo 드론 위치 리셋
        gz_cmd = [
            'gz', 'service',
            '-s', '/world/default/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'name:"x500_depth_0" position:{x:0 y:0 z:0} orientation:{w:1}'
        ]
        try:
            subprocess.run(gz_cmd, check=True)
            self.get_logger().info("📍 Gazebo에서 드론 위치 (0,0,0)으로 이동 완료")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Gazebo 위치 초기화 실패: {e}")

        # Landing pad 위치 리셋
        ranges = [(0.5, 1.0), (-1.0, -0.5)]
        x = random.uniform(*random.choice(ranges))
        y = random.uniform(*random.choice(ranges))

        set_pose_cmd = [
            'gz', 'service',
            '-s', '/world/default/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name:"landing_pad" position:{{x:{x} y:{y} z:0.0}} orientation:{{w:1}}'
        ]
        try:
            subprocess.run(set_pose_cmd, check=True)
            print(f"🟩 랜딩패드 위치 이동: x={x:.2f}, y={y:.2f}", flush=True)
        except subprocess.CalledProcessError as e:
            print(f"❌ 랜딩패드 위치 이동 실패: {e}", flush=True)

    # def takeoff(self):
    #     self._publish_mode()
    #     self._publish_position_setpoint(self.takeoff_z)
    #     return


    def timer_callback(self):
        self._publish_mode()
        #
        # if self.cooldown_start_time is not None:
        #     if time.time() - self.cooldown_start_time < self.cooldown_duration:
        #         return
        #     else:
        #         self.cooldown_start_time = None
        #
        # if self.counter < 10:
        #     self.counter += 1
        #     return
        #
        # if self.counter == 10 and not self.offboard_enabled:
        #     self.engage_offboard()
        #     self.arm()
        #     return
        #
        # if self.episode_start_time is None:
        #     if self.pos.z < self.takeoff_z + 0.3:
        #         self.get_logger().info("🛫 이륙 감지됨! 에피소드 타이머 시작")
        #         self.episode_start_time = time.time()
        #     else:
        #         self._publish_position_setpoint(self.takeoff_z)
        #     return
        #
        # elapsed = time.time() - self.episode_start_time
        # if elapsed <= self.episode_duration:
        #     self._publish_position_setpoint(self.takeoff_z)
        # else:
        #     self.get_logger().info('🔁 10초 호버링 종료. 리셋 시작')
        #     self.disarm()
        #     self.reset_drone_position()
        #     self.counter = -1
        #     self.offboard_enabled = False
        #     self.armed = False
        #     self.episode_start_time = None
        #     self.cooldown_start_time = time.time()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardControlForRL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Exception occurred: {e}")
