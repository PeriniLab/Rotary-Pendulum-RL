import pybullet as p
import numpy as np
import os
import time
import pybullet_data
import copy
import gymnasium as gym
from gymnasium import spaces

class PyBulletPendulumEnv(gym.Env):
    """
    PyBullet Rotary Pendulum
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode="human"):
        super(PyBulletPendulumEnv, self).__init__()
        """
        Initialize the PyBullet Rotary Pendulum environment

        Args:
            render (bool, optional): Whether to render the environment. Defaults to True.

        Returns:
            None
        """

        self.render_mode = render_mode
        # Initialize PyBullet
        if render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.806)
        # move camera to focus on the robot
        p.resetDebugVisualizerCamera(cameraDistance=0.4, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0.1])
        # Load the plane and pendulum URDF
        self.planeId = p.loadURDF("plane.urdf")
        self.load_pendulum_urdf()

        # Define other environment parameters
        self.name = "PyBulletPendulum"
        self.nbJoint = 1
        self.num_state = 2
        self.action = 0.0
        self.range_actions = np.array([-1.0, 1.0])
        self.n_actions = 101
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(self.num_state,), dtype=np.float32)
        self.action_space = spaces.Box(low=self.range_actions[0], high=self.range_actions[1], shape=(1,), dtype=np.float32)
        self.motorAngle = 0.0
        self.terminated = False
        self.truncated = False
        self.info = {}
        self.iterCount = 0
        self.maxIter = 1500
        self.omega_max = 10.0
        self.episode_reward = 0.0
        
        # variable to store angles of one episode
        self.episode_angles = []

    def load_pendulum_urdf(self):
        """
        Load the pendulum URDF into the environment.

        Args:
            None

        Returns:
            None
        """

        cubeStartPos = [0, 0, 0]
        cubeStartOrientation = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
        curr_dir = os.path.abspath(os.path.dirname(__file__))
        robot_urdf = 'Rotary_Pendulum_URDF.urdf'
        # Construct the path to the URDF file
        urdf_path = os.path.join(curr_dir, '..', '..', '..', 'simulation', 'urdf', robot_urdf)
        self.robotId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation,
                                #   flags=p.URDF_USE_INERTIA_FROM_FILE,
                                  useFixedBase=True
                                  )

        # Define joint indices as per your URDF structure
        self.motor_joint_idx = [p.getJointInfo(self.robotId, i)[1] for i in range(p.getNumJoints(self.robotId))].index(b'Revolute_3')
        self.bar_joint_idx = [p.getJointInfo(self.robotId, i)[1] for i in range(p.getNumJoints(self.robotId))].index(b'Revolute_5')

        # Define real robot parameters
        self.steps_per_rev = 3200
        self.max_speed_steps_per_sec = 4000.0
        # Calculate radians per step
        self.radians_per_step = (2 * np.pi) / self.steps_per_rev
        # Calculate max speed in radians per second [rad/s]
        self.max_motor_speed = self.max_speed_steps_per_sec * self.radians_per_step
        # Admissible motor angle range [deg]
        self.motor_angle_range = [-150, 150]
        self.out_of_range = False

        # Compensation angles for the URDF
        self.motor_compensation_angle = 0.400
        self.bar_compensation_angle = -0.264

    def reset(self, seed=None, options=None):
        """
        Reset the environment to a random state

        Args:
            None

        Returns:
            state (np.array): [bar_angle, bar_angular_velocity]
        """

        super().reset(seed=seed, options=options)
        # Reset the episode angles
        self.episode_angles = []
        self.episode_reward = 0.0
        self.terminated = False
        # Send command to pendulum to reset to random position
        self.send_fake_serial([0, 1])

        # get the state from the pendulum
        self.observation_space, self.motorAngle, self.terminated = self.get_state()

        # Reset iteration count
        self.iterCount = 0
        self.info = {"episode": {"r": 0.0, "l": self.iterCount}}
    
        return self.observation_space.astype(np.float32), self.info
    
    def step(self, action):
        """
        Take a step in the environment

        Args:
            action (float): Motor speed percentage [-100, 100]

        Returns:
            state (np.array): [bar angle, bar angular velocity]
        """

        # multiply the action by 100 to get the percentage
        self.action = action*100.0
        # Send action to pendulum over serial
        self.send_fake_serial([self.action, 0])
        # Read state and episode done flag from serial
        self.observation_space, self.motorAngle, self.terminated = self.get_state()
        
        p.stepSimulation()

        # Store the angles of the episode for reward penalty
        self.episode_angles.append(self.observation_space[0])
        
        # Calculate reward
        reward = self.calculate_reward(self.observation_space)
        self.episode_reward += reward
        self.iterCount += 1
        self.reset_policy(self.maxIter)
        self.info = {"episode": {"r": self.episode_reward, "l": self.iterCount}}

        # return normalized_state, reward, self.done
        return self.observation_space.astype(np.float32), reward, self.terminated, self.truncated, self.info
    
    def calculate_reward(self, state):
        """
        Calculate the reward for the current state

        Args:
            state (np.array): [bar angle, bar angular velocity]

        Returns:
            reward (float): Reward for the current state
        """

        # Constants to scale the bar and motor angle penalties
        ANGLE_WEIGHT = 1.0
        VELOCITY_WEIGHT = 0.1
        MOTOR_ANGLE_WEIGHT = 0.001
        ACTION_WEIGHT = 0.001

        # Calculate the angle penalty
        angle_penalty = ANGLE_WEIGHT * (state[0]) ** 2
        # Calculate the velocity penalty
        velocity_penalty = VELOCITY_WEIGHT * (state[1]) ** 2
        # Calculate the motor angle penalty
        # motor_angle_penalty = MOTOR_ANGLE_WEIGHT * (self.motorAngle/self.motor_angle_range[1]) ** 2
        # Calculate the action penalty
        action_penalty = ACTION_WEIGHT * (self.action/100) ** 2

        # Calculate the reward
        reward = - (angle_penalty + velocity_penalty)

        # NEW REWARD FUNCTION
        # reward range [-1, 0]
        # angle_target = 0.0
        # angular_velocity_target = 0.0
        # motor_angle_target = 0.0

        # reward = -1/2 * (np.abs(state[0] - angle_target)/np.pi + np.abs(self.motorAngle - motor_angle_target)/self.motor_angle_range[1])
        # reward = - 1/2 * (np.abs(state[0] - angle_target) + np.abs(state[1] - angular_velocity_target))
        # if the episode is done with enough iterations
        # if self.iterCount > int(self.maxIter/2) and self.done:
        #     # if the average of the bar angles is less than 90 degrees
        #     if np.abs(np.mean(self.episode_angles)) < np.deg2rad(90):
        #         reward += 100.0

        # if the episode is done with not enough iterations
        # if self.iterCount < int(self.maxIter/10) and self.terminated:
        #     # if the motor angle is out of range
        #     if self.out_of_range:
        #         reward -= 2000.0
        
        return reward
    
    def reset_policy(self, reset_count=200):
        """
        Policy to reset the environment

        Args:
            reset_count (int, optional): Number of iterations to wait before resetting the system. Defaults to 200.
        
        Returns:
            None
        """

        if self.iterCount >= reset_count:
            self.terminated = True

    def send_fake_serial(self, command):
        """
        Send a command to the pendulum, simulating a fake serial connection

        Args:
            command (list): [motor speed percentage, episode done flag]

        Returns:
            None
        """

        motor_speed_percentage = command[0]
        episode_done = command[1]

        if episode_done:
            self.terminated = True
            self.reset_robot(mode="random")
        else:
            self.terminated = False
            # Calculate the motor speed in steps per second
            motor_speed = motor_speed_percentage * self.max_motor_speed / 100.0
            # set the motor velocity
            p.setJointMotorControl2(bodyUniqueId=self.robotId,
                                    jointIndex=self.motor_joint_idx,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=motor_speed,
                                    )

        # time.sleep(0.1)
    
    def get_state(self):
        """
        Read the state from the pendulum, simulating a fake serial connection

        Args:
            None

        Returns:
            state (np.array): [bar angle, bar angular velocity]
            motor_angle (float): Motor angle in degrees
            done (bool): Episode done flag
        """

        # Get the bar angle
        bar_angle = p.getJointState(self.robotId, self.bar_joint_idx)[0] + self.bar_compensation_angle
        # Get bar angular velocity
        bar_angular_velocity = p.getJointState(self.robotId, self.bar_joint_idx)[1]
        # Get the motor angle
        motor_angle = np.rad2deg(p.getJointState(self.robotId, self.motor_joint_idx)[0] + self.motor_compensation_angle)

        # Map the motor angle to the correct range
        if motor_angle > self.motor_angle_range[1] or motor_angle < self.motor_angle_range[0]:
            self.out_of_range = True
        else:
            self.out_of_range = False
            
        # Adjusting the bar angle to map correctly
        bar_angle = bar_angle % (2 * np.pi)  # Normalize the angle to be within 0 to 2π
        if bar_angle > np.pi:
            bar_angle -= 2 * np.pi  # Adjust angles greater than π to be between -π to π
        
        if bar_angle > 0:
            bar_angle = np.pi - bar_angle
        elif bar_angle < 0:
            bar_angle = -np.pi - bar_angle

        # round the states to 4 decimal places
        bar_angle = round(bar_angle/np.pi, 4)
        bar_angular_velocity = round(bar_angular_velocity/self.omega_max, 4)
        motor_angle = round(motor_angle, 4)

        return np.array([bar_angle, bar_angular_velocity]), motor_angle, self.out_of_range
    
    def reset_robot(self, mode="random"):
        """
        Reset the robot state

        Args:
            mode (str, optional): Mode to reset the robot. Defaults to "random".

        Returns:
            state (np.array): [bar angle, bar angular velocity]
        """

        if mode == "random":
            # Reset the robot to a random position
            bar_angle = np.random.uniform(-np.pi, np.pi)
            bar_angular_velocity = np.random.uniform(-self.omega_max, self.omega_max)
            motor_angle = np.deg2rad(np.random.uniform(self.motor_angle_range[0], self.motor_angle_range[1]))

            # Set the robot to the random position
            p.resetJointState(self.robotId, self.bar_joint_idx, targetValue=bar_angle)
            # p.resetJointState(self.robotId, self.motor_joint_idx, targetValue=motor_angle)
            p.resetJointState(self.robotId, self.motor_joint_idx, targetValue=-self.motor_compensation_angle)
            # set bar velocity with no force
            p.setJointMotorControl2(bodyUniqueId=self.robotId,
                                    jointIndex=self.bar_joint_idx,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=bar_angular_velocity,
                                    force=0
                                    )
        elif mode == "home":
            # Reset the robot to the home position
            p.resetJointState(self.robotId, self.bar_joint_idx, targetValue=-self.bar_compensation_angle)
            p.resetJointState(self.robotId, self.motor_joint_idx, targetValue=-self.motor_compensation_angle)

            # set bar velocity with no force
            p.setJointMotorControl2(bodyUniqueId=self.robotId,
                                    jointIndex=self.bar_joint_idx,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=0,
                                    force=0
                                    )
        
        return self.get_state()[0]
    
    def render(self, fps=240.0):
        """
        Render the pendulum in PyBullet

        Args:
            fps (float, optional): Number of frames per second. Defaults to 240.0.

        Returns:
            None
        """
        p.stepSimulation()
        # time.sleep(1./fps)
    
    def close(self):
        """
        Close the PyBullet connection

        Args:
            None

        Returns:
            None
        """
        p.disconnect()
        
    