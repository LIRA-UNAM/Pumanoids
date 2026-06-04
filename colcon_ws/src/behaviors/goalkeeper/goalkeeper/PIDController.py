class PIDController:
    def __init__(self, kp, ki, kd, output_min=-0.3, output_max=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt

        if not self.initialized:
            derivative = 0.0
            self.initialized = True
        else:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min

        return output