import numpy as np
import serial
import time
from .SerialReader import SerialReader

class RealPendulum:
    """
    Real rotary pendulum with ESP32
    """
    def __init__(self, port, baudrate):
        """
        Initialize the environment.
        
        Args:
            port (str): The serial port to connect to.
            baudrate (int): The baudrate to use for the serial connection.

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
        self.name = "RealPendulum"
        self.nbJoint = 1
        self.num_state = 2
        self.action = 0.0
        self.state = np.zeros(self.num_state)
        self.motorAngle = 0.0
        self.done = False
        self.iterCount = 0
        self.maxIter = 1000
        self.omega_max = 10.0
        self.range_actions = np.array([-100.0, 100.0])

        # variable to store angles of one episode
        self.episode_angles = []
    
    def reset(self):
        """
        Reset the environment to the initial state.

        Args:
            None

        Returns:
            state (np.array): [bar angle, bar angular velocity]
        """

        # Reset the episode angles
        self.episode_angles = []

        # Send command to pendulum to go to home position.
        self.send_serial("0,1")
        # Wait for the pendulum to report it has finished resetting.
        while (1):
            self.state, self.motorAngle, self.done = self.reader.get_state()
            if not self.done:
                break

        # Reset iteration count
        self.iterCount = 0
        # normalized_state = self.normalize_state(self.state)
        return self.state
    
    def step(self, action):
        """
        Take a step in the environment

        Args:
            action (float): Motor speed percentage [-100, 100]

        Returns:
            state (np.array): [bar angle, bar angular velocity]
            reward (float): Reward for the current state
            done (bool): Whether the episode is done or not
        """

        # Send action to pendulum over serial
        self.send_serial(f"{action},0")
        # Read state and episode done flag from serial
        self.state, self.motorAngle, self.done = self.reader.get_state()

        # Store the angles of the episode for reward penalty
        self.episode_angles.append(self.state[0])
        
        # Calculate reward
        normalized_state = self.normalize_state(self.state)
        reward = self.calculate_reward(normalized_state)
        self.iterCount += 1
        self.reset_policy(self.maxIter)

        return self.state, reward, self.done

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
    
    def normalize_state(self, state):
        """
        Normalize the state to [-1, 1]

        Args:
            state (np.array): [bar angle, bar angular velocity]

        Returns:
            normalized_state (np.array): [bar angle norm, bar angular velocity norm]
        
        """
        normalized_angle = state[0] / np.pi  # Normalizes to [-1, 1]
        normalized_velocity = state[1] / self.omega_max  # Normalizes to [-1, 1]

        return np.array([normalized_angle, normalized_velocity])
    
    def reset_policy(self, reset_count=200):
        """
        Policy to reset the environment

        Args:
            reset_count (int, optional): Number of iterations to wait before resetting the system. Defaults to 200.
        
        Returns:
            None
        """

        if self.iterCount > reset_count:
            self.done = True
    
    def calculate_reward(self, normalized_state):
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
        angle_penalty = ANGLE_WEIGHT * (normalized_state[0] ** 2)
        # Penalize the angular velocity to be minimized
        velocity_penalty = VELOCITY_WEIGHT * (normalized_state[1] ** 2)

        # Penalize the motor angle to be minimized
        normalized_motor_angle = self.motorAngle / 180.0
        motor_angle_penalty = MOTOR_ANGLE_WEIGHT * (normalized_motor_angle ** 2)

        # Penalize the action to be minimized
        normalized_action = self.action / np.max(self.range_actions)
        action_penalty = ACTION_WEIGHT * (normalized_action ** 2)

        # Reward is higher when penalties are lower
        reward = -(angle_penalty + velocity_penalty + motor_angle_penalty + action_penalty)

        # Penalize the reward if the average angle of the episode is close to pi
        # after 3/4 of the maximum iterations
        if self.iterCount > self.maxIter*3/4:
            if np.abs(np.mean(self.episode_angles)) < (np.pi-0.8):
                reward-=100.0
        # if self.done:
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
