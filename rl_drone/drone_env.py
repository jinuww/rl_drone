import numpy as np
import gym
from gym import spaces
import time
import rclpy
from rl_drone.offboard_control import OffboardControlForRL

class DroneEnv(gym.Env):
    def __init__(self):
        super(DroneEnv, self).__init__()

        rclpy.init()
        self.node = OffboardControlForRL()

        # 관측 공간 (예: 3D 위치 + 속도 + 원 위치 + 거리)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)

        # 행동 공간 (예: TrajectorySetpoint: [x, y, z, yaw])
        self.action_space = spaces.Box(low=np.array([-5, -5, -5, -np.pi]),
                                       high=np.array([5, 5, 0, np.pi]),
                                       dtype=np.float32)

        self.max_steps = 40
        self.current_step = 0
        self.total_reward = 0.0

    def reset(self):
        self.node.disarm()
        self.node.reset_drone_position()
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)
        self.node.engage_offboard()

        # 목표 고도까지 position setpoint를 계속 보냄
        while True:
            pos = self.node.pos  # 혹은 self.node.get_state_vector() 등
            dz = abs(pos.z - self.node.takeoff_z)
            dx = abs(pos.x - 0.0)
            dy = abs(pos.y - 0.0)
            if dx < 0.1 and dy < 0.1 and dz < 0.1:
                print(f"[RESET] 드론이 이륙 고도({self.node.takeoff_z:.2f}m)에 도달. 에피소드 시작!", flush=True)
                break
            self.node._publish_position_setpoint(self.node.takeoff_z)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        self.current_step = 0
        self.total_reward = 0.0
        return np.array(self.node.get_state_vector(), dtype=np.float32)

    def step(self, action):
        x, y, z, yaw = action
        self.node._publish_position_setpoint(z)

        # ROS 콜백 처리
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.05)

        obs = np.array(self.node.get_state_vector(), dtype=np.float32)
        reward = self._compute_reward(obs, action)

        # 리워드 누적
        self.total_reward += reward
        self.current_step += 1

        done = self.current_step >= self.max_steps
        info = {}

        # 매 스텝 로그
        print(f"[STEP] action={action}, reward={reward:.3f}, total_reward={self.total_reward:.3f}, done={done}", flush=True)

        # 에피소드 종료 시 한 번 출력
        if done:
            print(f"*** Episode finished. Total Reward: {self.total_reward:.3f} ***", flush=True)

        return obs, reward, done, info

    def _compute_reward(self, obs, action):
        cx, cy, dist = obs[-3:]
        err_center = np.sqrt((cx - 320) ** 2 + (cy - 240) ** 2)
        reward = -0.01 * err_center - 0.5 * abs(dist - 1.0)
        return reward

    def render(self, mode='human'):
        pass

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
