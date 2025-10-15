from rl_drone.drone_env import DroneEnv
from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise
import numpy as np

def main():
    env = DroneEnv()

    n_actions = env.action_space.shape[0]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    model = DDPG(
        policy="MlpPolicy",
        env=env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log="./ddpg_tensorboard/"
    )

    model.learn(total_timesteps=10000, tb_log_name="ddpg_run")
    model.save("ddpg_drone")
    env.close()

if __name__ == '__main__':
    main()
