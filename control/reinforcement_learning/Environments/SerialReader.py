import threading
import time
import numpy as np

class SerialReader(threading.Thread):
    def __init__(self, ser, simulation=False):
        threading.Thread.__init__(self)
        self.ser = ser
        self.simulation = simulation
        self.daemon = True  # Set thread as daemon so it will end with the main program
        self.state = np.zeros(2)
        self.motorAngle = 0.0
        self.episode_done = False
        self.lock = threading.Lock()  # Lock to prevent simultaneous read/write of state

    def run(self):
        while True:
            if not self.simulation:
                self.read_serial()
            else:
                self.read_sim()
    
    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode().strip()
            parts = line.split(',')
            if len(parts) == 4:
                with self.lock:
                    self.state = np.array([float(parts[0]), float(parts[1])])
                    self.motorAngle = float(parts[2])
                    self.episode_done = bool(float(parts[3]))
                    # print(self.state, self.motorAngle, self.episode_done)
    
    def read_sim(self):
        with self.lock:
            print("TO IMPLEMENT")
            
    def get_state(self):
        time.sleep(0.1)
        with self.lock:
            return self.state.copy(), self.motorAngle, self.episode_done
