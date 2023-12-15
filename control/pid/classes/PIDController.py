class PIDController:
    """
    PID Controller class
    """
    def __init__(self, Kp, Ki, Kd):
        """
        Initialize PID controller gains

        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
        """
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        """
        Compute PID control signal

        Args:
            error (float): Error signal

        Returns:
            control_input (float): PID control signal
        """

        self.integral += error
        derivative = error - self.prev_error
        control_input = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        return control_input
    
    def reset(self):
        """
        Reset PID controller
        """

        self.prev_error = 0
        self.integral = 0