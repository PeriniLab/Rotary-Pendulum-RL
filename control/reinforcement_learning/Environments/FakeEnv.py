import numpy as np

class FakeEnv:
    """
    Fake environment for testing purposes
    """
    def __init__(self, nbJoint=1):
        """
        Initialize the fake environment.

        Args:
            nbJoint (int): The number of joints to simulate.

        Returns:
            None
        """

        self.name = "FakeEnv"
        self.nbJoint = nbJoint
        if nbJoint == 1:
            self.num_state = 2
            self.name = "1-fakeenv"
        else:
            self.num_state = 2*nbJoint
            self.name = str(nbJoint)+"-fakeenv"
        
        self.x = np.zeros(self.num_state)
        self.vmax = 8.0
        self.iterCount = 0
        self.maxIter = 1000
        self.range_actions = np.array([-100.0, 100.0])
        self.done = False

    def reset(self):
        """
        Reset the environment to the initial state (random)

        Args:
            None

        Returns:
            state (np.array): list of joint angles and velocities
        """

        for i in range(self.num_state):
            if i%2==0:
                self.x[i] = np.random.uniform(-np.pi, np.pi)
            else:
                self.x[i] = np.random.uniform(-self.vmax, self.vmax)
        self.iterCount = 0
        self.done = False
        return self.x
    
    def step(self, action):
        """
        Take a step in the environment (random)

        Args:
            action (float): The action to take (it is not used)

        Returns:
            state (np.array): list of joint angles and velocities
            reward (float): The reward for the action taken (it is not used)
            done (bool): Whether the episode is done or not
        """

        for i in range(self.num_state):
            if i%2==0:
                self.x[i] = np.random.uniform(-np.pi, np.pi)
            else:
                self.x[i] = np.random.uniform(-self.vmax, self.vmax)
        self.iterCount += 1
        if self.iterCount >= self.maxIter:
            self.done = True
        return self.x, np.random.uniform(-1, 1), self.done
    
    def render(self, debug=False):
        """
        Print the current state

        Args:
            debug (bool): Whether to print the state or not

        Returns:
            None
        """
        
        if debug:
            print(self.x)

