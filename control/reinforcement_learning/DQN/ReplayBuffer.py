import random
import numpy as np
from collections import deque

class ReplayBuffer:
    """
    Stores and retrieves gameplay experiences
    """

    def __init__(self, size):
        self.gameplay_experiences = deque(maxlen=size)
    
    def store_tuple(self, state, action, reward, new_state, done):
        """
        Store the experience in the replay buffer
        """
        self.gameplay_experiences.append((state, action, reward, new_state, done))

    def sample_batch(self, batch_size):
        """
        Sample a random batch of experiences from the replay buffer
        """
        random_sample = random.sample(self.gameplay_experiences, batch_size)
        states, actions, rewards, new_states, dones = map(np.asarray, zip(*random_sample))
        return states, actions, rewards, new_states, dones
