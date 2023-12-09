import numpy as np

class FakeEnv:
    """
    Fake environment for testing purposes
    """
    def __init__(self, nbJoint=1):
        
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
        self.done = False

    def reset(self):
        """
        Reset the environment to the initial state (random)
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
    
    def render(self):
        print(self.x)
        pass

