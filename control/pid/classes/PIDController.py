class PIDController:
    '''
    PID Controller class
    '''
    def __init__(self, Kp, Ki, Kd):
        '''
        Initialize PID controller gains
        '''
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        '''
        Compute PID control signal
        '''

        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output