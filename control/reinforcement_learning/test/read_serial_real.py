from ..Environments import RealPendulumEnv as real

# Create the environment
env = real.RealPendulum("COM3", 115200)
env.reset()
direction = 1
speed = 50  # 50% of max speed

while True:
    env.step(direction*speed)
    if env.done:
        # change direction at every reset
        direction *= -1
        env.reset()