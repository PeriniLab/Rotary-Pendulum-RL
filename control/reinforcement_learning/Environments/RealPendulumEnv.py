import numpy as np
import serial
import time
from .SerialReader import SerialReader
import gymnasium as gym
from gymnasium import spaces

class RealPendulumEnv(gym.Env):
    """
    Real rotary pendulum with ESP32
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, port, baudrate, render_mode="human"):
        super(RealPendulumEnv, self).__init__()
        """
        Initialize the environment.
        
        Args:
            port (str): The serial port to connect to.
            baudrate (int): The baudrate to use for the serial connection.
            render_mode (str, optional): The render mode. Defaults to "human".

        Returns:
            None
        """

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.reader = SerialReader(self.ser, simulation=False)
        self.reader.start()
        self.render_mode = render_mode
        self.name = "RealPendulum"
        self.nbJoint = 1
        self.num_state = 2
        self.action = 0.0
        self.motorAngle = 0.0
        self.terminated = False
        self.truncated = False
        self.iterCount = 0
        self.maxIter = 1000
        self.omega_max = 10.0
        self.range_actions = np.array([-1.0, 1.0])
        self.range_observation = np.array([-1.0, 1.0])
        self.observation_space = spaces.Box(low=self.range_observation[0], high=self.range_observation[1], shape=(self.num_state,), dtype=np.float32)
        self.action_space = spaces.Box(low=self.range_actions[0], high=self.range_actions[1], shape=(1,), dtype=np.float32)
        # variable to store angles of one episode
        self.episode_angles = []
    
    def reset(self, seed=None, options=None):
        """
        Reset the environment to the initial state.

        Args:
            None

        Returns:
            state (np.array): [bar angle, bar angular velocity]
            info (dict): Episode information
        """

        super().reset(seed=seed, options=options)

        # Reset the episode angles
        self.episode_angles = []

        # Send command to pendulum to go to home position.
        self.send_serial("0,1")
        # Wait for the pendulum to report it has finished resetting.
        while (1):
            self.observation_space, self.motorAngle, self.terminated = self.reader.get_state()
            if not self.terminated:
                break

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
            reward (float): Reward for the current state
            terminated (bool): Whether the episode is done or not
            truncated (bool): Whether the episode is truncated or not
            info (dict): Episode information
        """

        # Send action to pendulum over serial
        self.send_serial(f"{action*100},0")
        self.action = action
        # Read state and episode done flag from serial
        self.observation_space, self.motorAngle, self.terminated = self.reader.get_state()

        # Store the angles of the episode for reward penalty
        self.episode_angles.append(self.state[0])
        
        # Calculate reward
        reward = self.calculate_reward(self.observation_space)
        self.episode_reward += reward
        self.iterCount += 1
        self.reset_policy(self.maxIter)
        self.info = {"episode": {"r": self.episode_reward, "l": self.iterCount}}

        return self.observation_space.astype(np.float32), reward, self.terminated, self.truncated, self.info

    def send_serial(self, command):
        """
        Send a command to the pendulum over serial

        Args:
            command (str): [motor speed percentage, reset flag]

        Returns:
            None
        """

        self.ser.write(f"{command}\n".encode())
        # time.sleep(0.1)
    
    def reset_policy(self, reset_count=200):
        """
        Policy to reset the environment

        Args:
            reset_count (int, optional): Number of iterations to wait before resetting the system. Defaults to 200.
        
        Returns:
            None
        """

        if self.iterCount > reset_count:
            self.terminated = True
    
    def calculate_reward(self, state):
        """
        Calculate the reward for the current state

        Args:
            state (np.array): [bar angle, bar angular velocity]

        Returns:
            reward (float): Reward for the current state
        """

        # Constants to scale the angle and velocity penalties
        ANGLE_WEIGHT = 1.0
        VELOCITY_WEIGHT = 0.1
        MOTOR_ANGLE_WEIGHT = 1.0
        ACTION_WEIGHT = 0.01

        # Penalize the angle to be minimized
        angle_penalty = ANGLE_WEIGHT * (state[0] ** 2)
        # Penalize the angular velocity to be minimized
        velocity_penalty = VELOCITY_WEIGHT * (state[1] ** 2)

        # Penalize the motor angle to be minimized
        motor_angle = self.motorAngle / 180.0
        motor_angle_penalty = MOTOR_ANGLE_WEIGHT * (motor_angle ** 2)

        # Penalize the action to be minimized
        action_penalty = ACTION_WEIGHT * (self.action ** 2)

        # Reward is higher when penalties are lower
        reward = -(angle_penalty + velocity_penalty + motor_angle_penalty + action_penalty)

        # Penalize the reward if the average angle of the episode is close to pi
        # after 3/4 of the maximum iterations
        if self.iterCount > self.maxIter*3/4:
            if np.abs(np.mean(self.episode_angles)) < (np.pi-0.8):
                reward-=100.0
        # if self.terminated:
        #     if self.iterCount < self.maxIter*1/10:
        #         reward-=100.0
        return reward

    def render(self, camera=False):
        """
        Render the state (optional), e.g. display the video stream
        """
        if camera:
            print("Connect the camera to the pendulum and display the video stream.")

    def close(self):
        """
        Close the serial connection

        Args:
            None

        Returns:
            None
        """

        self.ser.close()
