import numpy as np

class PurePursuitController:
    def __init__(self, wheelbase_length, lookahead_distance):
        """
        Initialize the Pure Pursuit controller.
        
        :param wheelbase_length: Length between the front and rear axles of the vehicle (L).
        :param lookahead_distance: The fixed distance from the vehicle to the lookahead point (Ld).
        """
        self.L = wheelbase_length
        self.Ld = lookahead_distance
        self.max_steering_angle = 0.45

    def transform_to_vehicle_frame(self, x, y, theta, waypoint):
        """
        Transform waypoint coordinates to the vehicle's coordinate frame.
        
        :param x: Vehicle's x position.
        :param y: Vehicle's y position.
        :param theta: Vehicle's heading (orientation) angle in radians.
        :param waypoint: The (x, y) coordinates of the waypoint.
        :return: Transformed (x, y) coordinates of the waypoint in the vehicle's coordinate frame.
        """
        dx = waypoint[0] - x
        dy = waypoint[1] - y
        
        x_transformed = dx * np.cos(-theta) - dy * np.sin(-theta)
        y_transformed = dx * np.sin(-theta) + dy * np.cos(-theta)
        
        return x_transformed, y_transformed

    def calculate_steering_angle(self, x, y, theta, waypoint):
        """
        Calculate the steering angle to follow the path using the Pure Pursuit algorithm.
        
        :param x: Vehicle's x position.
        :param y: Vehicle's y position.
        :param theta: Vehicle's heading (orientation) angle in radians.
        :param waypoint: The (x, y) coordinates of the lookahead waypoint.
        :return: Required steering angle in radians.
        """
        x_transformed, y_transformed = self.transform_to_vehicle_frame(x, y, theta, waypoint)
        
        # Calculate the curvature using the Pure Pursuit formula
        kappa = (2 * y_transformed) / (self.Ld ** 2)
        
        # Calculate the steering angle
        steering_angle = np.arctan(kappa * self.L)
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        
        return steering_angle
    

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        """
        Initializes the PID Controller with gains and timestep.
        :param kp: Proportional gain.
        :param ki: Integral gain.
        :param kd: Derivative gain.
        :param dt: Time step for discrete differentiation and integration.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0
        self.integral = 0

    def reset(self):
        """Resets the PID controller state."""
        self.previous_error = 0
        self.integral = 0

    def control(self, setpoint, measured_value):
        """
        Calculates the control action from a PID controller for a given setpoint and measured value.
        :param setpoint: The desired target value.
        :param measured_value: The current measured value.
        :return: Control input.
        """
        # Calculate error
        error = setpoint - measured_value

        # Proportional term
        P = self.kp * error

        # Integral term
        self.integral += error * self.dt
        I = self.ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        D = self.kd * derivative

        # Update previous error
        self.previous_error = error

        # Calculate total control input
        u = P + I + D

        return u