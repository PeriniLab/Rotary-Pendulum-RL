from ...reinforcement_learning.Environments import RealPendulumEnv as real
from ...reinforcement_learning.Environments import PyBulletPendulumEnv as pybullet
from ..classes.PIDController import PIDController
import numpy as np

real_pendulum = False

# Example usage
pid = PIDController(Kp=18, Ki=0.01, Kd=0.001)
K_motor = 0.0
desired_bar_angle = 0
desired_bar_velocity = 0
desired_motor_angle = 0

if real_pendulum:
    # initialize RealPendulum environment
    env = real.RealPendulumEnv("COM3", 115200)
else:
    # initialize PyBulletPendulum environment
    env = pybullet.PyBulletPendulumEnv(render_mode='human')

env.maxIter = 1_000_000
# reset environment to home position
env.reset()
# get initial observation
observation, reward, done, _, _ = env.step(0)

while True:

    # compute error and control signal 
    error = (desired_bar_angle - observation[0]) + (desired_bar_velocity - observation[1]) # + K_motor * (desired_motor_angle - env.motorAngle)
    control_signal = pid.compute(error)
    print(f"Bar Angle: {observation[0]}, Bar Velocity: {observation[1]}, Motor Angle: {env.motorAngle}, Control Signal: {control_signal}")
    # print(f"Error: {error}, Control Signal: {control_signal}")

    # step environment with pid control signal
    observation, reward, done, _, _ = env.step(control_signal)
    # render environment
    env.render()
    # reset pid controller and position if pendulum goes out of bounds
    if done:
        env.reset()
        pid.integral = 0
        pid.prev_error = 0