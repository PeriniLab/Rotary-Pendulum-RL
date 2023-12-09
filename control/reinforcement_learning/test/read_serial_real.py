from control.reinforcement_learning.Environments import RealPendulum as real

env = real.RealPendulum("COM3", 115200)
env.reset()
direction = 1
speed = 50  # 50% of max speed
while True:
    env.step(direction*speed)
    if env.done:
        direction *= -1
        env.reset()