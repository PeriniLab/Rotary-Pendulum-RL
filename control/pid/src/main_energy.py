from ...reinforcement_learning.Environments import RealPendulumEnv as real
from ...reinforcement_learning.Environments import PyBulletPendulumEnv as pybullet
from ..classes.EnergyController import EnergyController
import numpy as np
import time

real_pendulum = False

# Example usage
energy_controller = EnergyController()
K_motor = 0.0
desired_bar_angle = 0
desired_bar_velocity = 0
desired_motor_angle = 0

if real_pendulum:
    # initialize RealPendulum environment
    env = real.RealPendulumEnv("COM3", 115200)
else:
    # initialize PyBulletPendulum environment
    env = pybullet.PyBulletPendulumEnv(render=True)

env.maxIter = 1_000_000
# reset environment to home position
env.reset()
# get initial observation
observation, reward, done = env.step(0)

while True:

    # # if the angle is between -pi and 0, map it to 2pi to pi
    # if observation[0] < 0:
    #     observation[0] = observation[0] + 2 * np.pi
    # compute error and control signal 
    control_signal = energy_controller.control(observation[0], observation[1])
    print(f"Bar Angle: {observation[0]}, Bar Velocity: {observation[1]}, Motor Angle: {env.motorAngle}, Control Signal: {control_signal}")
    # print(f"Error: {error}, Control Signal: {control_signal}")

    # step environment with pid control signal
    observation, reward, done = env.step(control_signal)
    # render environment
    env.render()
    # reset pid controller and position if pendulum goes out of bounds
    if env.done:
        env.reset()
        energy_controller.reset()