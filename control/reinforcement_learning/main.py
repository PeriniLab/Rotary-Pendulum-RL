from control.reinforcement_learning.Environments import RealPendulum as real
from control.reinforcement_learning.Environments import PyBulletPendulum as pb
from control.reinforcement_learning.Environments import FakeEnv as fake
from control.reinforcement_learning.DQN.Agent import Agent

isFake = False
isPyBullet = True
isReal = False

train = True
plot_colormaps = False

# select the environment
if isFake:
    env = fake.FakeEnv(1)
elif isPyBullet:
    env = pb.PyBulletPendulum()
elif isReal:
    env = real.RealPendulum("COM3", 115200)
else:
    raise Exception("No environment selected!")

# create the agent
dqn_agent = Agent(env)

# train or evaluate the agent
if train:
    dqn_agent.train_model(render=True, plot=True, verbose=True, soft_start=False)
else:
    dqn_agent.evaluate_model(episodes=10, swingUp=False, render=True, verbose=True, final=False)

# plot the value function and policy
if plot_colormaps:
    dqn_agent.plot_value_policy('2D', resolution=50, final=False)