from stable_baselines3.common.env_checker import check_env
from sb3_contrib import TQC
from stable_baselines3 import DDPG
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from ..Environments import RealPendulumEnv as real
from ..Environments import PyBulletPendulumEnv as pb
import numpy as np

isReal = False

# select the environment
if isReal:
    env = real.RealPendulumEnv("COM3", 115200)
else:
    env = pb.PyBulletPendulumEnv(render_mode="human")

# check custom environment and output additional warnings if needed
# check_env(env, warn=True)

env = make_vec_env(lambda: env, n_envs=1)
# The noise objects for DDPG
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

# initialize the agent
model = TQC("MlpPolicy", env, verbose=1, tensorboard_log="./tensorboard/", top_quantiles_to_drop_per_net=2, learning_starts=300_000, batch_size=512, buffer_size=1_000_000, train_freq=5)
# model = DDPG("MlpPolicy", env, verbose=1, tensorboard_log="./tensorboard/")
# define a callback function to save the model every 1000 steps
def save_model_callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    # save the agent every 1000 steps
    if (model.num_timesteps + 1) % 1000 == 0:
        model.save("tqc_pendulum_4")
    return True
# train the agent
model.learn(total_timesteps=10_000_000, log_interval=10, progress_bar=True, callback=save_model_callback)

# save the agent
model.save("tqc_pendulum_4")

# model = TQC.load("tqc_pendulum_4")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render("human")